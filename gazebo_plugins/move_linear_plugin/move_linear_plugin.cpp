#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class MoveLinearPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
    {
      this->model = _model;
      this->initial_pose = this->model->WorldPose();
      this->last_update_time = common::Time::GetWallTime();
      this->direction = 1; // 처음엔 +방향

      // 매 시뮬레이션 프레임마다 호출될 함수 등록
      this->update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MoveLinearPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      // 시간 계산
      common::Time current_time = common::Time::GetWallTime();
      double dt = (current_time - this->last_update_time).Double();
      this->last_update_time = current_time;

      // 현재 위치
      ignition::math::Pose3d pose = this->model->WorldPose();
      double dy = speed * dt * direction;

      // Y축만 업데이트, 나머지는 초기값 유지
      double new_y = pose.Pos().Y() + dy;

      ignition::math::Pose3d new_pose = this->initial_pose;
      new_pose.Pos().Y() = new_y;

      // 이동 거리 계산
      double moved = new_y - this->initial_pose.Pos().Y();

      // 방향 전환 조건
      if (std::abs(moved) >= max_distance)
      {
        direction *= -1;

        if (moved > 0)
          new_pose.Pos().Y() = this->initial_pose.Pos().Y() + max_distance;
        else
          new_pose.Pos().Y() = this->initial_pose.Pos().Y() - max_distance;
      }

      // 위치 적용 (Z값은 항상 초기값 유지됨)
      this->model->SetWorldPose(new_pose);

    }

  private:
    physics::ModelPtr model;
    event::ConnectionPtr update_connection;
    ignition::math::Pose3d initial_pose;
    common::Time last_update_time;
    int direction; // +1 또는 -1

    double speed = 0.07;         // m/s
    double max_distance = 0.5;  // ±1m 이동
  };

  GZ_REGISTER_MODEL_PLUGIN(MoveLinearPlugin)
}

