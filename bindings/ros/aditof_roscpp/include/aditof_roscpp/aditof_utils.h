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
#ifndef ADITOF_UTILS_H
#define ADITOF_UTILS_H

#include <aditof/camera.h>
#include <glog/logging.h>
#include <mutex>

enum class ModeTypes { NONE, mode3, mode7, mode10 };

std::string *parseArgs(int argc, char **argv);
std::shared_ptr<aditof::Camera> initCamera(std::string *arguments);
void startCamera(const std::shared_ptr<aditof::Camera> &camera);
void stopCamera(const std::shared_ptr<aditof::Camera> &camera);
void setFrameType(const std::shared_ptr<aditof::Camera> &camera,
                  const std::string &type);
void getAvailableFrameType(const std::shared_ptr<aditof::Camera> &camera,
                           std::vector<std::string> &availableFrameTypes);
void getCameraDataDetails(const std::shared_ptr<aditof::Camera> &camera,
                          aditof::CameraDetails &details);
void enableCameraDepthCompute(const std::shared_ptr<aditof::Camera> &camera,
                              const bool value);
void setMode(const std::shared_ptr<aditof::Camera> &camera,
             const std::string &mode);
void setCameraRevision(const std::shared_ptr<aditof::Camera> &camera,
                       const std::string rev);
void setIrGammaCorrection(const std::shared_ptr<aditof::Camera> &camera,
                          float gamma);
void applyNoiseReduction(const std::shared_ptr<aditof::Camera> &camera,
                         int threshold);
void disableNoiseReduction(const std::shared_ptr<aditof::Camera> &camera);
void getNewFrame(const std::shared_ptr<aditof::Camera> &camera,
                 aditof::Frame **frame);
uint16_t *getFrameData(aditof::Frame **frame, const std::string &dataType);
aditof::IntrinsicParameters
getIntrinsics(const std::shared_ptr<aditof::Camera> &camera);
int getRangeMax(const std::shared_ptr<aditof::Camera> &camera);
int getRangeMin(const std::shared_ptr<aditof::Camera> &camera);
void irTo16bitGrayscale(uint16_t *frameData, int width, int height);
enum ModeTypes intToMode(int var);

#endif // ADITOF_UTILS_H
