/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "sthal_SoundTriggerHw"

#include <android/binder_status.h>
#include <cutils/properties.h>
#include <dlfcn.h>
#include <log/log.h>
#include <soundtriggerhw/SoundTriggerHw.h>
#include <utils/CoreUtils.h>
#include <utils/PalToAidlConverter.h>
#include "PalApi.h"

using android::OK;

//Returns retVal incase of invalid session
#define CHECK_VALID_SESSION(session, handle, retVal)                 \
    ({                                                               \
        if (session == nullptr) {                                    \
            ALOGE("%s: invalid handle %d", __func__, handle);       \
            return CoreUtils::halErrorToAidl(retVal);                \
        }                                                            \
    })

namespace aidl::android::hardware::soundtrigger3 {

SoundTriggerHw::SoundTriggerHw()
{
    ALOGI("%s: ", __func__);
    mSoundTriggerInitDone = true;
}

SoundTriggerHw::~SoundTriggerHw()
{
    ALOGI("%s: ", __func__);
}

ScopedAStatus SoundTriggerHw::registerGlobalCallback(
    const std::shared_ptr<ISoundTriggerHwGlobalCallback> &callback)
{
    int status = 0;
    pal_param_resources_available_t param_resource_avail;

    ALOGV("%s: Enter", __func__);

    mGlobalCallback = callback;
    param_resource_avail.callback = (void*)&onResourcesAvailable;
    param_resource_avail.cookie = (uint64_t)this;

    status = pal_set_param(PAL_PARAM_ID_ST_RESOURCES_AVAILABLE,
                          (void*)&param_resource_avail,
                          sizeof(pal_param_resources_available_t));
    if (status) {
        ALOGE("%s: failed to set paramID for resources available, status %d",
            __func__, status);
    }

    ALOGI("%s: Exit, status %d", __func__, status);
    return CoreUtils::halErrorToAidl(status);
}

std::shared_ptr<SoundTriggerSession> SoundTriggerHw::getSession(int32_t handle)
{
    std::shared_ptr<SoundTriggerSession> client = nullptr;

    std::lock_guard<std::mutex> lock(mMutex);
    if (mSessions.find(handle) != mSessions.end()) {
        client = mSessions[handle];
    } else {
        ALOGE("%s: client not found for handle %d", __func__, handle);
    }
    return client;
}

void SoundTriggerHw::addSession(std::shared_ptr<SoundTriggerSession> &session)
{
    std::lock_guard<std::mutex> lock(mMutex);
    mSessions[session->getSessionHandle()] = session;

    ALOGI("%s: handle %d, sessions %d", __func__, session->getSessionHandle(), mSessions.size());
}

void SoundTriggerHw::removeSession(int32_t handle)
{
    std::lock_guard<std::mutex> lock(mMutex);
    mSessions.erase(handle);
}

ScopedAStatus SoundTriggerHw::getProperties(Properties *aidlProperties)
{
    int status = 0;
    struct pal_st_properties *palProperties = nullptr;
    size_t size = 0;

    ALOGV("%s: Enter", __func__);

    status = pal_get_param(PAL_PARAM_ID_GET_SOUND_TRIGGER_PROPERTIES,
                         (void **)&palProperties, &size, nullptr);

    if (status || !palProperties || size < sizeof(struct pal_st_properties)) {
        ALOGE("%s: query properties from pal failed, status %d", __func__, status);
        return CoreUtils::halErrorToAidl(status);
    }

    PalToAidlConverter::convertProperties(palProperties, *aidlProperties);

    auto st_session = std::make_shared<SoundTriggerSession>(0, nullptr);
    aidlProperties->supportedModelArch = st_session->getModuleVersion();

    ALOGI("%s: Exit properties %s, status %d", __func__,
                            aidlProperties->toString().c_str(), status);
    return CoreUtils::halErrorToAidl(status);
}

ScopedAStatus SoundTriggerHw::loadSoundModel(
    const SoundModel &model,
    const std::shared_ptr<ISoundTriggerHwCallback> &callback,
    int32_t *handle)
{
    int status = 0;

    ALOGI("%s: Enter", __func__);

    *handle = nextUniqueModelId();
    auto st_session = std::make_shared<SoundTriggerSession>(*handle, callback);
    status = st_session->loadSoundModel(model);

    if (status) {
        ALOGE("%s: Failed to load sound model with handle %d, status %d",
                                                    __func__, *handle, status);
        handle = nullptr;
        return CoreUtils::halErrorToAidl(status);
    }

    addSession(st_session);

    ALOGI("%s: Exit handle %d, status %d", __func__, *handle, status);
    return ndk::ScopedAStatus::ok();
}

ScopedAStatus SoundTriggerHw::loadPhraseSoundModel(
    const PhraseSoundModel &model,
    const std::shared_ptr<ISoundTriggerHwCallback> &callback,
    int32_t *handle)
{
    int status = 0;

    ALOGI("%s: Enter", __func__);

    *handle = nextUniqueModelId();
    auto st_session = std::make_shared<SoundTriggerSession>(*handle, callback);
    status = st_session->loadPhraseSoundModel(model);

    if (status) {
        ALOGE("%s: Failed to load phrase sound model with handle %d, status %d",
                                                    __func__, *handle, status);
        handle = nullptr;
        return CoreUtils::halErrorToAidl(status);
    }

    addSession(st_session);

    ALOGI("%s: Exit handle %d, status %d", __func__, *handle, status);
    return ndk::ScopedAStatus::ok();
}

ScopedAStatus SoundTriggerHw::unloadSoundModel(int32_t handle)
{
    int status = 0;

    ALOGI("%s: Enter handle %d", __func__, handle);

    auto st_session = getSession(handle);
    CHECK_VALID_SESSION(st_session, handle, STATUS_INVALID_OPERATION);

    status = st_session->unloadSoundModel();
    if (status != 0) {
        ALOGE("%s: Failed to unload sound model with handle %d, status %d",
                                                        __func__, handle, status);
        return CoreUtils::halErrorToAidl(status);
    }

    removeSession(handle);

    ALOGI("%s: Exit handle %d, status %d", __func__, handle, status);
    return CoreUtils::halErrorToAidl(status);
}

ScopedAStatus SoundTriggerHw::startRecognition(
    int32_t modelHandle,
    int32_t deviceHandle,
    int32_t ioHandle,
    const RecognitionConfig &config)
{
    int status = 0;

    ALOGI("%s: Enter handle %d", __func__, modelHandle);

    auto st_session = getSession(modelHandle);
    CHECK_VALID_SESSION(st_session, modelHandle, STATUS_INVALID_OPERATION);

    status = st_session->startRecognition(deviceHandle, ioHandle, config);
    if (status != 0) {
        ALOGE("%s: Failed to start recognition model with handle %d, status %d",
                                                       __func__, ioHandle, status);
        return CoreUtils::halErrorToAidl(status);
    }

    ALOGI("%s: Exit handle %d, status %d", __func__, modelHandle, status);
    return CoreUtils::halErrorToAidl(status);
}

ScopedAStatus SoundTriggerHw::stopRecognition(int32_t handle)
{
    int status = 0;

    ALOGI("%s: Enter handle %d", __func__, handle);

    auto st_session = getSession(handle);
    CHECK_VALID_SESSION(st_session, handle, STATUS_INVALID_OPERATION);

    status = st_session->stopRecognition();
    if (status != 0) {
        ALOGE("%s: Failed to stop recognition model with handle %d, status %d",
                                                      __func__, handle, status);
        return CoreUtils::halErrorToAidl(status);
    }

    ALOGI("%s: Exit handle %d, status %d", __func__, handle, status);
    return CoreUtils::halErrorToAidl(status);
}

// TODO implement this API
ScopedAStatus SoundTriggerHw::forceRecognitionEvent(int32_t handle)
{
    int status = -ENOSYS;

    ALOGI("%s: unsupported API", __func__);
    return CoreUtils::halErrorToAidl(status);
}

ScopedAStatus SoundTriggerHw::queryParameter(
    int32_t handle,
    ModelParameter modelParams,
    std::optional<ModelParameterRange> *aidlRange)
{
    int status = -ENOSYS;

    ALOGI("%s: unsupported API", __func__);
    return CoreUtils::halErrorToAidl(status);
}

ScopedAStatus SoundTriggerHw::getParameter(
    int32_t handle,
    ModelParameter modelParams,
    int32_t *aidlParameters)
{
    int status = -ENOSYS;

    ALOGI("%s: unsupported API", __func__);
    return CoreUtils::halErrorToAidl(status);
}

ScopedAStatus SoundTriggerHw::setParameter(
    int32_t handle,
    ModelParameter modelParams,
    int32_t value)
{
    int status = -ENOSYS;

    ALOGI("%s: unsupported API", __func__);
    return CoreUtils::halErrorToAidl(status);
}

void SoundTriggerHw::onResourcesAvailable(uint64_t cookie)
{
    SoundTriggerHw *hw = (SoundTriggerHw *)cookie;

    if (hw)
        hw->mGlobalCallback->onResourcesAvailable();

    ALOGI("%s: Exit", __func__);
}

} // namespace aidl::android::hardware::soundtrigger3
