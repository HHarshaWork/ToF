/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "connections/usb/usb_sensor_enumerator.h"

using namespace aditof;

Status UsbSensorEnumerator::searchSensors() {
    // TO DO: implement this when enabling macos support

    return Status::OK;
}

Status UsbSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<DepthSensorInterface>> & /*depthSensors*/) {

    // TO DO: implement this when enabling macos support

    return Status::OK;
}

Status UsbSensorEnumerator::getStorages(
    std::vector<std::shared_ptr<StorageInterface>> & /*storages*/) {

    // TO DO: implement this when enabling macos support

    return Status::OK;
}

Status UsbSensorEnumerator::getTemperatureSensors(
    std::vector<std::shared_ptr<TemperatureSensorInterface>>
        & /*temperatureSensors*/) {

    // TO DO: implement this when enabling macos support

    return Status::OK;
}

aditof::Status
UsbSensorEnumerator::getUbootVersion(std::string &uBootVersion) const {
    uBootVersion = m_uBootVersion;
    return aditof::Status::OK;
}

aditof::Status
UsbSensorEnumerator::getKernelVersion(std::string &kernelVersion) const {
    kernelVersion = m_kernelVersion;
    return aditof::Status::OK;
}

aditof::Status
UsbSensorEnumerator::getRfsVersion(std::string &rfsVersion) const {
    rfsVersion = m_rfsVersion;
    return aditof::Status::OK;
}

aditof::Status
UsbSensorEnumerator::getSdkVersion(std::string &sdkVersion) const {
    sdkVersion = m_sdkVersion;
    return aditof::Status::OK;
}