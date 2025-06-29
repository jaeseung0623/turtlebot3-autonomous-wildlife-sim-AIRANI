// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/common/common.hh>

// namespace gazebo
// {
//   class MoveBoxPlugin : public ModelPlugin
//   {
//   public:
//     void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
//     {
//       this->model = _model;
//       this->updateConnection = event::Events::ConnectWorldUpdateBegin(
//         std::bind(&MoveBoxPlugin::OnUpdate, this));
//     }

//     void OnUpdate()
//     {
//       ignition::math::Vector3d vel(0.1, 0, 0);  // x축으로 움직이기
//       this->model->SetLinearVel(vel);
//     }

//   private:
//     physics::ModelPtr model;
//     event::ConnectionPtr updateConnection;
//   };

//   GZ_REGISTER_MODEL_PLUGIN(MoveBoxPlugin)
// }
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Vector3.hh>
#include <chrono>

namespace gazebo
{
  class MoveBoxPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
    {
      this->model = _model;

      // 시작 시간 저장
      this->start_time = std::chrono::steady_clock::now();

      // 업데이트 루프 연결
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MoveBoxPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      // 현재 시간 계산
      auto now = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = now - this->start_time;
      double t = elapsed.count();  // 경과 시간 (초)

      // 타원 궤적 파라미터
      double a = 0.5;    // x축 반지름
      double b = 0.3;   // y축 반지름
      double omega = -0.05; // 각속도 (라디안/초)

      // 반시계방향 회전 (속도 방향 반전)
      double vx =  a * omega * sin(omega * t);
      double vy = -b * omega * cos(omega * t);

      this->model->SetLinearVel(ignition::math::Vector3d(vy, vx, 0));
      this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));

    }

  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;

    std::chrono::steady_clock::time_point start_time;
  };

  GZ_REGISTER_MODEL_PLUGIN(MoveBoxPlugin)
}
