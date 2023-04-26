/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "sthal_SharedMemoryWrapper"

#include <android-base/macros.h>
#include <cutils/ashmem.h>
#include <log/log.h>
#include <sys/mman.h>
#include <utils/SharedMemoryWrapper.h>

#include "PalApi.h"

namespace aidl::android::hardware::soundtrigger3 {

SharedMemoryWrapper::SharedMemoryWrapper(int fd, int size) :
    mSharedMemoryFd(fd),
    mExpectedMmapSize(size)
{
    if ((mSharedMemoryFd < 0) || !ashmem_valid(mSharedMemoryFd) ||
        (mExpectedMmapSize != ashmem_get_size_region(mSharedMemoryFd))) {
            LOG_ALWAYS_FATAL("Invalid SharedMemory fd %d", __func__, mSharedMemoryFd);
    } else {
        ALOGI("%s: SharedMemory fd %d, size %d", __func__, mSharedMemoryFd, size);

        mSharedMemory = mmap(NULL, mExpectedMmapSize, PROT_READ, MAP_SHARED, mSharedMemoryFd, 0);
        if (mSharedMemory == MAP_FAILED) {
            LOG_ALWAYS_FATAL("Failed to map SharedMemory fd %d", __func__, mSharedMemoryFd);
        }
    }
}

SharedMemoryWrapper::~SharedMemoryWrapper()
{
    if (mSharedMemory != nullptr && munmap(mSharedMemory, mExpectedMmapSize) < 0) {
        ALOGE("%s: unmap failed for fd %d", __func__, mSharedMemoryFd);
    }
}

const uint8_t* SharedMemoryWrapper::data()
{
    return reinterpret_cast<const uint8_t *>(mSharedMemory);
}

} // namespace aidl::android::hardware::soundtrigger3
