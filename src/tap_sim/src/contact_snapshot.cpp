// Publish physics collision snapshots, including empty samples while airborne.
#include <array>
#include <chrono>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/transport/Node.hh>

namespace tap_sim
{
class ContactSnapshot : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
  gz::sim::Entity model_ = gz::sim::kNullEntity;
  gz::transport::Node node_;
  std::array<gz::transport::Node::Publisher, 2> pubs_;
public:
  void Configure(const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    model_ = entity;
    pubs_[0] = node_.Advertise<gz::msgs::Contacts>("/left_wheel_contacts");
    pubs_[1] = node_.Advertise<gz::msgs::Contacts>("/right_wheel_contacts");
  }
  void PostUpdate(const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {return;}
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(info.simTime);
    const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime - seconds);
    for (std::size_t side = 0; side < 2; ++side) {
      const std::string name = side == 0 ? "cad_left_wheel_link" : "cad_right_wheel_link";
      const auto link = gz::sim::Model(model_).LinkByName(ecm, name);
      if (link == gz::sim::kNullEntity) {continue;}
      const auto collision = gz::sim::Link(link).CollisionByName(ecm, name + "_collision_collision");
      const auto data = ecm.Component<gz::sim::components::ContactSensorData>(collision);
      // Missing physics data is not an airborne observation. The stock Contact
      // system creates this component from the sensor's collision reference.
      if (!data) {continue;}
      auto msg = data->Data();
      msg.mutable_header()->mutable_stamp()->set_sec(seconds.count());
      msg.mutable_header()->mutable_stamp()->set_nsec(nanos.count());
      pubs_[side].Publish(msg);
    }
  }
};
}  // namespace tap_sim
GZ_ADD_PLUGIN(tap_sim::ContactSnapshot, gz::sim::System,
  tap_sim::ContactSnapshot::ISystemConfigure, tap_sim::ContactSnapshot::ISystemPostUpdate)
