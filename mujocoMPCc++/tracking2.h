// Copyright 2022 DeepMind Technologies Limited
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

#ifndef MJPC_TASKS_HUMANOID_TRACKING2_TASK_H_
#define MJPC_TASKS_HUMANOID_TRACKING2_TASK_H_

#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {
namespace humanoid {

class Tracking2 : public Task {
 public:
  class ResidualFn2 : public mjpc::BaseResidualFn {
   public:
    explicit ResidualFn2(const Tracking2* task, int current_mode = 0,
                        double reference_time = 0)
        : mjpc::BaseResidualFn(task),
          current_mode_(current_mode),
          reference_time_(reference_time) {}

    // ------------- Residuals for humanoid tracking task -------------
    //   Number of residuals:
    //     Residual (0): Joint vel: minimise joint velocity
    //     Residual (1): Control: minimise control
    //     Residual (2-11): Tracking position: minimise tracking position error
    //         for {root, head, toe, heel, knee, hand, elbow, shoulder, hip}.
    //     Residual (11-20): Tracking velocity: minimise tracking velocity error
    //         for {root, head, toe, heel, knee, hand, elbow, shoulder, hip}.
    //   Number of parameters: 0
    // ----------------------------------------------------------------

    ///////user data[]
    //
    //data->userdata[0]=sensor_pos[0]; --- root world position for mocap and residual
    //data->userdata[1]=sensor_pos[1];
    //d->userdata[2]=(double)key_index_0; --last anim index
    //d->userdata[3]=-d->time;  ---time offset
    //d->userdata[4]=residual_.current_mode_;  ---last anim mode for stand up auto set anim mode



    //--------
    void Residual(const mjModel* model, const mjData* data,
                  double* residual) const override;
   private:
    friend class Tracking2;
    int current_mode_;
    double reference_time_;

    
    int prevkeyframe;
    double pelvisPosSensor[3]={0,0,0} ;
    double delX=0.0;
    double delY=0.0;
    double delZ=0.0;
  };

  Tracking2() : residual_(this) {}

  // --------------------- Transition for humanoid task ------------------------
  //   Set `data->mocap_pos` based on `data->time` to move the mocap sites.
  //   Linearly interpolate between two consecutive key frames in order to
  //   smooth the transitions between keyframes.
  // ---------------------------------------------------------------------------
  void TransitionLocked(mjModel* model, mjData* data) override;

  std::string Name() const override;
  std::string XmlPath() const override;
  std::string XmlPath2() const override;


  void InputDataFunc(const mjModel* model,  mjData* data)  override;

 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn2>(this, residual_.current_mode_,
                                        residual_.reference_time_);
  }
  ResidualFn2* InternalResidual() override { return &residual_; }

 private:
  // int current_mode_;
  // double reference_time_;
  ResidualFn2 residual_;
};

}  // namespace humanoid
}  // namespace mjpc

#endif  // MJPC_TASKS_HUMANOID_TRACKING2_TASK_H_
