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
#ifndef PUBLISHER_FACTORY_H
#define PUBLISHER_FACTORY_H

#include "../../../../sdk/include/aditof/camera.h"
#include "message_factory.h"
#include <aditof_utils.h>
#include <dynamic_reconfigure/server.h>
#include <memory>
#include <ros/ros.h>
#include <typeinfo>
#include <vector>

class PublisherFactory {
  public:
    PublisherFactory();
    void createNew(ModeTypes mode, ros::NodeHandle nHandle,
                   const std::shared_ptr<aditof::Camera> &camera,
                   aditof::Frame **frame);
    void updatePublishers(const std::shared_ptr<aditof::Camera> &camera,
                          aditof::Frame **frame);
    void deletePublishers(const std::shared_ptr<aditof::Camera> &camera);
    void setDepthFormat(const int val);

  private:
    std::vector<ros::Publisher> img_publishers;
    std::vector<std::shared_ptr<AditofSensorMsg>> imgMsgs;

  public:
    ModeTypes m_currentMode = ModeTypes::NONE;
    bool m_enableDepthCompute = 1;
};

#endif // PUBLISHER_FACTORY_H
