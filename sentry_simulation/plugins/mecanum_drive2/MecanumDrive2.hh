// Copyright 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef GZ_SIM_SYSTEMS_MECANUM_DRIVE_HH
#define GZ_SIM_SYSTEMS_MECANUM_DRIVE_HH

#include "../gazebo_compat.hh"
#include <memory>

namespace SENTRY_GZ
{
    namespace SENTRY_SIM
    {
        namespace systems
        {
            class MecanumDrive2Private;
            class GZ_SIM_VISIBLE MecanumDrive2
                : public SENTRY_GZ::SENTRY_SIM::System,
                  public ISystemConfigure,
                  public ISystemPreUpdate,
                  public ISystemPostUpdate
            {
            public:
                MecanumDrive2();
                ~MecanumDrive2() override = default;

            public:
                void Configure(const Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               EntityComponentManager &_ecm,
                               EventManager &_eventMgr) override;
                void PreUpdate(const SENTRY_GZ::SENTRY_SIM::UpdateInfo &_info,
                               SENTRY_GZ::SENTRY_SIM::EntityComponentManager &_ecm) override;
                void PostUpdate(const SENTRY_GZ::SENTRY_SIM::UpdateInfo &_info,
                                const SENTRY_GZ::SENTRY_SIM::EntityComponentManager &_ecm) override;

            private:
                std::unique_ptr<MecanumDrive2Private> dataPtr;
            };
        } // namespace systems
    }     // namespace SENTRY_SIM
} // namespace SENTRY_GZ

#endif //GZ_SIM_SYSTEMS_MECANUM_DRIVE_HH