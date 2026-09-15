#include "robot_control/JumpController.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_control {
namespace {
float smooth(float t) { t=std::clamp(t,0.F,1.F); return t*t*(3-2*t); }
JointArray blend(const JointArray& a,const JointArray& b,float t) {
  JointArray q{};
  for (std::size_t i=0;i<q.size();++i) q[i]=a[i]+t*(b[i]-a[i]);
  return q;
}
JointArray hermite(const JointArray& a,const JointArray& velocity,const JointArray& b,
                   float elapsed,float duration) {
  const float s=std::clamp(elapsed/duration,0.F,1.F);
  auto q=blend(a,b,smooth(s));
  const float tangent=duration*s*(1.F-s)*(1.F-s);
  for(auto j:{0,1,3,4}) q[j]+=tangent*velocity[j];
  return q;
}
}
JumpController::JumpController():JumpController(Config{}) {}
JumpController::JumpController(Config config):config_(config) {
  if(config.plan) {
    if(!std::isfinite(config.plan_preparation_wait_s) ||
       config.plan_preparation_wait_s<2.F || config.plan_preparation_wait_s>10.F)
      throw std::invalid_argument("Invalid planned preparation wait");
    if(!config.plan->valid() || config.max_joint_speed_rad_s<28.F ||
       config.landing_world_frame || config.smooth_phase_transitions ||
       config.thrust_track_measured_knee || config.thrust_knee_force_control ||
       config.landing_compression || config.thrust_wheel_control ||
       config.thrust_hip_offset_rad!=0.F || config.flight_hip_rate_kp!=0.F ||
       config.flight_hip_rate_kd!=0.F)
      throw std::invalid_argument("Invalid optimized plan or conflicting trajectory owners");
    for(float tau:{config.plan_prep_hip_torque_nm,config.plan_prep_knee_torque_nm})
      if(!std::isfinite(tau)||std::abs(tau)>5.F)
        throw std::invalid_argument("Invalid plan preparation torque");
    const auto first=config.plan->sample(0.F);
    const auto last=config.plan->sample(2.F);
    if(std::abs(first.knee_rad-config.crouch_knee_rad)>.001F ||
       std::abs(last.knee_rad-config.landing_knee_rad)>.001F ||
       std::abs(first.hip_rad-neutralPose(first.knee_rad)[0])>.01F ||
       std::abs(last.hip_rad-neutralPose(last.knee_rad)[0])>.01F)
      throw std::invalid_argument("Plan endpoints differ from preparation/recovery poses");
  }
  if (!std::isfinite(config.thrust_hip_offset_rad)||std::abs(config.thrust_hip_offset_rad)>.15F)
    throw std::invalid_argument("Invalid thrust hip offset");
  if (!std::isfinite(config.thrust_wheel_speed_m_s)||std::abs(config.thrust_wheel_speed_m_s)>.35F)
    throw std::invalid_argument("Invalid thrust wheel speed");
  if (!std::isfinite(config.compression_knee_rad)||config.compression_knee_rad< -1.6F||
      config.compression_knee_rad>0.F||!std::isfinite(config.compression_duration_s)||
      config.compression_duration_s<.05F||config.compression_duration_s>.5F||
      (config.landing_compression&&config.compression_knee_rad>=config.landing_knee_rad))
    throw std::invalid_argument("Invalid landing compression");
  if(config.landing_compression&&config.compression_duration_s+config.landing_duration_s>=config.max_settle_s)
    throw std::invalid_argument("Landing compression leaves no settling window");
  if((config.landing_world_frame||config.smooth_phase_transitions)&&
      (config.flight_hip_rate_kp>0.F||config.flight_hip_rate_kd>0.F))
    throw std::invalid_argument("Conflicting flight posture controllers");
  if (!std::isfinite(config.flight_hip_rate_kp) || config.flight_hip_rate_kp < 0 ||
      !std::isfinite(config.flight_hip_rate_kd) || config.flight_hip_rate_kd < 0)
    throw std::invalid_argument("Invalid flight posture feedback");
  if (!std::isfinite(config.body_pitch_kp) || config.body_pitch_kp < 0 ||
      !std::isfinite(config.body_pitch_kd) || config.body_pitch_kd < 0 ||
      !std::isfinite(config.hip_attitude_torque_limit_nm) ||
      config.hip_attitude_torque_limit_nm < 0 || config.hip_attitude_torque_limit_nm > 5)
    throw std::invalid_argument("Invalid jump attitude feedback");
  for (float v : {config.crouch_duration_s,config.thrust_duration_s,config.max_thrust_s,
       config.max_flight_s,config.landing_duration_s,config.settle_duration_s,
       config.max_settle_s,config.grounded_duration_s,config.contact_debounce_s,
       config.tracking_tolerance_rad,config.max_pitch_rad,config.max_leg_torque_nm,
       config.max_wheel_torque_nm,config.max_joint_speed_rad_s,config.thrust_force_n}) {
    if (!std::isfinite(v)||v<=0) throw std::invalid_argument("Invalid jump duration/gain/limit");
  }
  if (config.max_leg_torque_nm>5 || config.max_wheel_torque_nm>.13F ||
      config.max_joint_speed_rad_s>28.2744F ||
      config.max_pitch_rad>.8F || config.max_thrust_s<config.thrust_duration_s ||
      config.max_settle_s<config.settle_duration_s)
    throw std::invalid_argument("Jump exceeds nominal bounds or invalid timeout");
  for (float k : {config.crouch_knee_rad,config.extension_knee_rad,config.landing_knee_rad})
    if (!std::isfinite(k)||k< -1.6F||k>.2F) throw std::invalid_argument("Jump knee outside provisional geometry");
  if (!(config.crouch_knee_rad<config.landing_knee_rad && config.landing_knee_rad<=0 &&
        config.extension_knee_rad>config.crouch_knee_rad))
    throw std::invalid_argument("Invalid jump pose ordering");
  target_=rest_pose_;
  last_command_=target_;
}
JointArray JumpController::neutralPose(float knee) {
  return landingPose(knee,0.F);
}
JointArray JumpController::landingPose(float knee,float pitch) {
  WholeBodyModel model;
  const float c=std::cos(pitch),s=std::sin(pitch);
  float lo=-.6F,hi=1.05F;
  for (int i=0;i<30;++i) {
    float m=(lo+hi)/2;
    const auto body=model.compute(m,knee,m,knee);
    if (c*body.com_offset_x_m+s*body.com_above_axle_m>0) hi=m; else lo=m;
  }
  return {(lo+hi)/2,knee,0,(lo+hi)/2,knee,0};
}
void JumpController::enter(Phase phase,const char* reason) {phase_=phase;reason_=reason;elapsed_=0;}
void JumpController::reset() {
  phase_=Phase::Idle;reason_="idle";elapsed_=grounded_=lost_contact_=landed_=settled_=0;
  last_request_=airborne_=false;last_ground_com_=takeoff_com_=apex_com_=0;crouch_contact_lost_=0;
  start_=target_=rest_pose_;
  last_command_=target_;
  thrust_finished_={};
  plan_time_s_=0.F;
  plan_origin_x_=previous_body_x_=previous_body_z_=0;
  plan_wheel_offset_=body_vx_=body_vz_=0;
  plan_contact_wait_s_=0;
  last_velocity_=start_velocity_={};
}
void JumpController::abort(const char* reason) { enter(Phase::Fault,reason); }
bool JumpController::active() const noexcept {
  return phase_!=Phase::Idle && phase_!=Phase::Complete && phase_!=Phase::Fault;
}
const char* JumpController::phaseName(Phase p) noexcept {
  switch(p) {
    case Phase::Idle:return "Idle"; case Phase::Crouch:return "Crouch";
    case Phase::Thrust:return "Thrust"; case Phase::Flight:return "Flight";
    case Phase::Land:return "Land";case Phase::Settle:return "Settle";
    case Phase::Complete:return "Complete";case Phase::Fault:return "Fault";
  }
  return "Fault";
}
const char* JumpController::safetyFault(const Input& in, bool check_dt) const {
  bool finite=(!check_dt || (std::isfinite(in.dt_s)&&in.dt_s>0&&in.dt_s<=.05F))&&
      std::isfinite(in.pitch_rad)&&std::isfinite(in.pitch_rate_rad_s)&&std::isfinite(in.com_z_m)&&
      std::isfinite(in.ground_speed_m_s);
  for(float q:in.position_rad) finite=finite&&std::isfinite(q);
  for(float q:in.velocity_rad_s) finite=finite&&std::isfinite(q);
  if(config_.plan) finite=finite&&std::isfinite(in.body_x_m)&&std::isfinite(in.body_z_m);
  if (!finite||!in.sensors_fresh||!in.contacts_fresh||in.stop||!in.eligible) return "safety_or_stale";
  if (in.operator_driving) return "operator_drive";
  if (std::abs(in.pitch_rad)>config_.max_pitch_rad) return "pitch_limit";
  for (auto h:{0,3}) if (in.position_rad[h]<-.65F || in.position_rad[h]>1.10F ||
      in.position_rad[h+1]<-1.75F || in.position_rad[h+1]>.30F) return "measured_joint_travel";
  return nullptr;
}
JumpController::Output JumpController::update(const Input& in) {
  const bool trigger=in.request&&!last_request_;last_request_=in.request;
  const char* fault=safetyFault(in);
  const bool grounded=in.left_contact&&in.right_contact;
  const bool safe=fault==nullptr;
  bool neutral=true;
  const auto& rest=rest_pose_;
  for(auto i:{0,1,3,4}) neutral=neutral&&std::abs(in.position_rad[i]-rest[i])<.08F&&
      std::abs(in.velocity_rad_s[i])<.3F;
  if (safe && grounded && std::abs(in.pitch_rad)<.15F &&
      std::abs(in.pitch_rate_rad_s)<.3F && !in.operator_driving) grounded_+=in.dt_s;
  else grounded_=0;
  if(active() && fault) abort(fault);
  if(!config_.enabled) {reason_="disabled";}
  else if(trigger&&(phase_==Phase::Idle||phase_==Phase::Complete)) {
    if(!safe||grounded_<config_.grounded_duration_s||!neutral) reason_="not_ready_grounded_neutral";
    else {
      start_=in.position_rad;target_=neutralPose(config_.crouch_knee_rad);
      last_command_=in.position_rad;
      last_velocity_=start_velocity_={};
      thrust_finished_={};
      plan_time_s_=0.F;
      previous_body_x_=in.body_x_m;previous_body_z_=in.body_z_m;
      body_vx_=body_vz_=0;
      plan_contact_wait_s_=0;
      lost_contact_=landed_=settled_=crouch_contact_lost_=0;airborne_=false;takeoff_com_=apex_com_=0;
      enter(Phase::Crouch,"preparing");
    }
  }
  if(active()) {
    if(config_.plan) {
      const float gain=in.dt_s/(.015F+in.dt_s);
      body_vx_+=gain*(float((in.body_x_m-previous_body_x_)/in.dt_s)-body_vx_);
      body_vz_+=gain*(float((in.body_z_m-previous_body_z_)/in.dt_s)-body_vz_);
      previous_body_x_=in.body_x_m;previous_body_z_=in.body_z_m;
    }
    if(config_.plan && phase_!=Phase::Crouch) {
      if(phase_==Phase::Flight) {
        const float touchdown=config_.plan->thrust_duration_s+config_.plan->flight_duration_s;
        plan_time_s_=std::min(touchdown,plan_time_s_+in.dt_s);
        if(plan_time_s_>=touchdown)plan_contact_wait_s_+=in.dt_s;
      } else plan_time_s_+=in.dt_s;
    }
    elapsed_+=in.dt_s;
    if(grounded) last_ground_com_=in.com_z_m;
    lost_contact_=!in.left_contact&&!in.right_contact?lost_contact_+in.dt_s:0;
    if(phase_==Phase::Crouch) {
      crouch_contact_lost_=grounded?0:crouch_contact_lost_+in.dt_s;
      if(crouch_contact_lost_>=config_.contact_debounce_s) abort("contact_lost_before_thrust");
      else if(grounded && elapsed_>=config_.crouch_duration_s) {
        float error=0;
        for(auto i:{0,1,3,4}) error=std::max(error,std::abs(in.position_rad[i]-target_[i]));
        bool plan_ready=true;
        if(config_.plan) {
          plan_ready=error<.005F && std::abs(in.pitch_rad)<.01F &&
              std::abs(in.pitch_rate_rad_s)<.05F;
          for(auto j:{0,1,3,4}) plan_ready=plan_ready&&std::abs(in.velocity_rad_s[j])<.3F;
          plan_ready=plan_ready&&std::abs(in.ground_speed_m_s)<.003F&&
              std::abs(.5F*(in.velocity_rad_s[2]+in.velocity_rad_s[5]))<.1F;
        }
        if(plan_ready&&error<config_.tracking_tolerance_rad && std::abs(in.pitch_rate_rad_s)<.3F) {
          if(config_.plan) {
            plan_origin_x_=in.body_x_m-config_.plan->samples[0].base_x_m;
            plan_wheel_offset_=.5F*(in.position_rad[2]+in.position_rad[5])-
                config_.plan->samples[0].wheel_position_rad;
          }
          start_=target_;target_=neutralPose(config_.extension_knee_rad);enter(Phase::Thrust,"extending");
        } else if(elapsed_>config_.crouch_duration_s+
            (config_.plan?config_.plan_preparation_wait_s:2.F)) abort("crouch_tracking_timeout");
      }
    } else if(phase_==Phase::Thrust) {
      if(lost_contact_>=config_.contact_debounce_s) {
        airborne_=true;takeoff_com_=last_ground_com_;apex_com_=in.com_z_m;
        start_=config_.smooth_phase_transitions?last_command_:in.position_rad;
        start_velocity_=last_velocity_;
        target_=neutralPose(config_.landing_knee_rad);enter(Phase::Flight,"measured_takeoff");
      } else if(elapsed_>config_.max_thrust_s) abort("no_takeoff");
    } else if(phase_==Phase::Flight) {
      apex_com_=std::max(apex_com_,in.com_z_m);
      landed_=(in.left_contact||in.right_contact)?landed_+in.dt_s:0;
      if(landed_>=config_.contact_debounce_s) {
        if(config_.plan && plan_time_s_<config_.plan->thrust_duration_s+
            config_.plan->flight_duration_s-.05F) {
          abort("plan_contact_timing");
        } else {
        start_=config_.smooth_phase_transitions?last_command_:in.position_rad;
        start_velocity_=last_velocity_;enter(Phase::Land,"measured_landing");
        if(config_.plan)plan_time_s_=config_.plan->thrust_duration_s+config_.plan->flight_duration_s;
        }
      }
      else if(elapsed_>config_.max_flight_s) abort("flight_timeout");
    } else if(phase_==Phase::Land) {
      bool tracked=std::abs(in.pitch_rad)<.15F&&std::abs(in.pitch_rate_rad_s)<.3F;
      for(auto j:{0,1,3,4}) tracked=tracked && std::abs(in.position_rad[j]-target_[j])<
          config_.tracking_tolerance_rad && std::abs(in.velocity_rad_s[j])<.5F;
      const float landing_time=config_.landing_duration_s+
          (config_.landing_compression?config_.compression_duration_s:0.F);
      const bool plan_done=!config_.plan || plan_time_s_>=config_.plan->samples[config_.plan->count-1].time_s;
      if(plan_done&&elapsed_>=landing_time&&grounded&&tracked) {
        start_=target_;target_=rest_pose_;enter(Phase::Settle,"recovering_balance");
      } else if(elapsed_>config_.max_settle_s) abort("landing_timeout");
    } else if(phase_==Phase::Settle) {
      float error=0;
      for(auto i:{0,1,3,4}) error=std::max(error,std::abs(in.position_rad[i]-target_[i]));
      bool slow=std::abs(in.ground_speed_m_s)<.02F;
      for(auto j:{0,1,3,4}) slow=slow&&std::abs(in.velocity_rad_s[j])<.3F;
      const bool stable=slow&&grounded&&std::abs(in.pitch_rad)<.15F&&std::abs(in.pitch_rate_rad_s)<.3F&&
          error<config_.tracking_tolerance_rad;
      settled_=stable?settled_+in.dt_s:0;
      if(settled_>=config_.settle_duration_s) enter(Phase::Complete,"landed_settled");
      else if(elapsed_>config_.max_settle_s) abort("settle_timeout");
    }
  }
  Output out;out.phase=phase_;out.reason=reason_;out.owns_pose=active();out.stop_motors=phase_==Phase::Fault;
  out.ready=config_.enabled&&safe&&neutral&&grounded_>=config_.grounded_duration_s&&
      (phase_==Phase::Idle||phase_==Phase::Complete);
  out.balance_wheels=phase_!=Phase::Flight;out.airborne_seen=airborne_;out.phase_elapsed_s=elapsed_;
  if(phase_==Phase::Thrust&&config_.thrust_wheel_control) {
    out.override_wheel_speed=true;
    out.wheel_speed_rad_s=config_.thrust_wheel_speed_m_s/geometry::wheel_radius_m;
  }
  if (phase_==Phase::Flight) out.leg_kp_scale=.3F;
  if (phase_==Phase::Land) out.leg_kp_scale=.3F+.7F*smooth(
      (elapsed_-(config_.landing_compression?config_.compression_duration_s:0.F))/config_.landing_duration_s);
  if (phase_==Phase::Thrust||phase_==Phase::Flight||phase_==Phase::Land) out.station_blend=0;
  if (phase_==Phase::Settle) out.station_blend=smooth(elapsed_/config_.settle_duration_s);
  out.position_rad=target_;
  const bool planned=config_.plan && (phase_==Phase::Thrust||phase_==Phase::Flight||phase_==Phase::Land);
  if(planned) {
    auto ref=config_.plan->sample(plan_time_s_);
    if(phase_==Phase::Flight) {
      const float rate=plan_contact_wait_s_>0.F?0.F:1.F;
      ref.hip_velocity_rad_s*=rate;ref.knee_velocity_rad_s*=rate;
      ref.pitch_rate_rad_s*=rate;ref.wheel_velocity_rad_s*=rate;
      ref.base_vx_m_s*=rate;ref.base_vz_m_s*=rate;
    }
    out.position_rad={ref.hip_rad,ref.knee_rad,0,ref.hip_rad,ref.knee_rad,0};
    out.velocity_rad_s={ref.hip_velocity_rad_s,ref.knee_velocity_rad_s,0,
                       ref.hip_velocity_rad_s,ref.knee_velocity_rad_s,0};
    const std::array<float,12> error{
      float(in.body_x_m-plan_origin_x_)-ref.base_x_m,float(in.body_z_m)-ref.base_z_m,
      in.pitch_rad-ref.pitch_rad,.5F*(in.position_rad[0]+in.position_rad[3])-ref.hip_rad,
      .5F*(in.position_rad[1]+in.position_rad[4])-ref.knee_rad,
      .5F*(in.position_rad[2]+in.position_rad[5])-plan_wheel_offset_-ref.wheel_position_rad,
      body_vx_-ref.base_vx_m_s,body_vz_-ref.base_vz_m_s,in.pitch_rate_rad_s-ref.pitch_rate_rad_s,
      .5F*(in.velocity_rad_s[0]+in.velocity_rad_s[3])-ref.hip_velocity_rad_s,
      .5F*(in.velocity_rad_s[1]+in.velocity_rad_s[4])-ref.knee_velocity_rad_s,
      .5F*(in.velocity_rad_s[2]+in.velocity_rad_s[5])-ref.wheel_velocity_rad_s};
    std::array<float,3> torque{ref.hip_torque_nm,ref.knee_torque_nm,ref.wheel_torque_nm};
    for(std::size_t motor=0;motor<3;++motor)
      for(std::size_t state=0;state<12;++state)
        torque[motor]-=ref.feedback_gain[12*motor+state]*error[state];
    const float hip=std::clamp(torque[0],-5.F,5.F),knee=std::clamp(torque[1],-5.F,5.F);
    out.feedforward_nm={hip,knee,0,hip,knee,0};
    out.leg_kp_scale=0.F;  // TVLQR owns position feedback; local D remains active.
    out.balance_wheels=false;
    out.override_wheel_speed=true;
    out.wheel_speed_is_relative=true;
    out.wheel_speed_rad_s=std::clamp(.5F*(in.velocity_rad_s[2]+in.velocity_rad_s[5])+
                                   std::clamp(torque[2],-.13F,.13F)/.02F,
                                   -41.887902F,41.887902F);
    if(phase_==Phase::Flight) {
      const float touchdown=config_.plan->thrust_duration_s+config_.plan->flight_duration_s;
      const float blend=smooth((plan_time_s_-(touchdown-.08F))/.08F);
      const float carrier=in.pitch_rate_rad_s+.5F*(in.velocity_rad_s[0]+
          in.velocity_rad_s[1]+in.velocity_rad_s[3]+in.velocity_rad_s[4]);
      // Match rolling contact speed before impact; the plant retains torque limits.
      const float rolling=std::clamp(body_vx_/geometry::wheel_radius_m-carrier,
                                    -41.887902F,41.887902F);
      out.wheel_speed_rad_s+=(rolling-out.wheel_speed_rad_s)*blend;
    }
    last_command_=out.position_rad;
    const float touchdown=config_.plan->thrust_duration_s+config_.plan->flight_duration_s;
    if((phase_==Phase::Thrust&&plan_time_s_>config_.plan->thrust_duration_s+.05F)||
       (phase_==Phase::Flight&&plan_time_s_<config_.plan->thrust_duration_s-.05F)||
       (phase_==Phase::Flight&&plan_contact_wait_s_>.05F)||
       (phase_==Phase::Land&&plan_time_s_<touchdown-.05F)) {
      abort("plan_contact_timing");
      out.phase=phase_;out.reason=reason_;out.owns_pose=false;out.stop_motors=true;
      out.override_wheel_speed=false;out.velocity_rad_s={};out.feedforward_nm={};
    }
  } else if(active()) {
    float duration=config_.landing_duration_s;
    if(phase_==Phase::Crouch) duration=config_.crouch_duration_s;
    if(phase_==Phase::Thrust) duration=config_.thrust_duration_s;
    if(phase_==Phase::Settle) duration=config_.settle_duration_s;
    out.position_rad=blend(start_,target_,smooth(elapsed_/duration));
    if(config_.smooth_phase_transitions&&(phase_==Phase::Flight||phase_==Phase::Land))
      out.position_rad=hermite(start_,start_velocity_,target_,elapsed_,duration);
    if(phase_==Phase::Land&&config_.landing_compression) {
      const auto compressed=neutralPose(config_.compression_knee_rad);
      if(elapsed_<config_.compression_duration_s)
        out.position_rad=config_.smooth_phase_transitions?
            hermite(start_,start_velocity_,compressed,elapsed_,config_.compression_duration_s):
            blend(start_,compressed,smooth(elapsed_/config_.compression_duration_s));
      else out.position_rad=blend(compressed,target_,
          smooth((elapsed_-config_.compression_duration_s)/config_.landing_duration_s));
    }
    if(phase_==Phase::Crouch||phase_==Phase::Thrust||phase_==Phase::Settle) {
      const float delta=config_.max_joint_speed_rad_s*in.dt_s;
      const float knee=std::clamp(out.position_rad[1],last_command_[1]-delta,last_command_[1]+delta);
      out.position_rad=neutralPose(knee);
    }
    if (phase_==Phase::Thrust && config_.thrust_track_measured_knee) {
      // The lightly loaded hip can outrun the force-limited knee. Coordinate
      // each hip with actual extension instead of the time-ahead knee target.
      for (auto h:{0,3}) out.position_rad[h]=neutralPose(
          std::clamp(in.position_rad[h+1],-1.6F,.2F))[h];
    }
    if (phase_==Phase::Thrust) for(auto h:{0,3})
      out.position_rad[h]=std::clamp(out.position_rad[h]+
          config_.thrust_hip_offset_rad*smooth(elapsed_/config_.thrust_duration_s),-.6F,1.05F);
    if (phase_==Phase::Flight &&
        (config_.flight_hip_rate_kp>0 || config_.flight_hip_rate_kd>0)) {
      // Integrating the hip reference avoids a fixed-pose spring cancelling
      // the internal reaction used to correct body attitude during flight.
      const float rate=std::clamp(config_.flight_hip_rate_kp*in.pitch_rad+
          config_.flight_hip_rate_kd*in.pitch_rate_rad_s,
          -config_.max_joint_speed_rad_s,config_.max_joint_speed_rad_s);
      for (auto h:{0,3}) out.position_rad[h]=std::clamp(
          last_command_[h]+rate*in.dt_s,-.6F,1.05F);
    }
    if (config_.landing_world_frame && phase_==Phase::Flight) {
      // Rotate COM-to-axle geometry into the ground frame. A body-neutral pose
      // places the wheels ahead of the COM when the body pitches backwards.
      for (auto h:{0,3}) {
        const float knee=std::clamp(out.position_rad[h+1],-1.6F,.2F);
        const auto world=landingPose(knee,in.pitch_rad);
        if(config_.smooth_phase_transitions)
          out.position_rad[h]+=smooth(elapsed_/duration)*(world[h]-neutralPose(knee)[h]);
        else out.position_rad[h]=world[h];
      }
    }
    for(auto i:{0,1,3,4}) {
      const float delta=config_.max_joint_speed_rad_s*in.dt_s;
      if(config_.smooth_phase_transitions)
        out.position_rad[i]=std::clamp(out.position_rad[i],i==0||i==3?-.6F:-1.6F,i==0||i==3?1.05F:.2F);
      out.position_rad[i]=std::clamp(out.position_rad[i],last_command_[i]-delta,last_command_[i]+delta);
      out.velocity_rad_s[i]=(out.position_rad[i]-last_command_[i])/in.dt_s;
      if(config_.smooth_phase_transitions&&elapsed_==0.F&&
          (phase_==Phase::Flight||phase_==Phase::Land)&&out.position_rad[i]==last_command_[i])
        out.velocity_rad_s[i]=std::clamp(start_velocity_[i],
            -config_.max_joint_speed_rad_s,config_.max_joint_speed_rad_s);
    }
    last_command_=out.position_rad;
    if(config_.plan&&phase_==Phase::Crouch) {
      const float gain=smooth((elapsed_/config_.crouch_duration_s-.8F)/.2F);
      for(auto h:{0,3}) {
        out.feedforward_nm[h]=gain*config_.plan_prep_hip_torque_nm;
        out.feedforward_nm[h+1]=gain*config_.plan_prep_knee_torque_nm;
      }
    }
    if(phase_==Phase::Thrust && (in.left_contact||in.right_contact)) {
      for(int side=0;side<2;++side) {
        if(!(side==0?in.left_contact:in.right_contact)) continue;
        const int h=side*3,k=h+1;
        const auto& thigh=geometry::origins[side*3+1];
        const auto& shin=geometry::origins[side*3+2];
        const float a=in.position_rad[h]+(config_.thrust_world_frame?in.pitch_rad:0.F);
        const float b=a+in.position_rad[k];
        const float shin_x=std::cos(b)*shin.x+std::sin(b)*shin.z;
        const float wheel_x=std::cos(a)*thigh.x+std::sin(a)*thigh.z+shin_x;
        // J_z=[-wheel_x,-shin_x]. Oppose upward ground reaction at foot.
        const float max_lever=std::max(std::abs(wheel_x),std::abs(shin_x));
        const float force=std::min(config_.thrust_force_n/2,
            config_.max_leg_torque_nm/std::max(max_lever,1e-5F));
        out.feedforward_nm[h]=wheel_x*force;
        out.feedforward_nm[k]=shin_x*force;
        if (config_.thrust_knee_force_control) {
          // Extend under the actuator's torque/speed envelope. A delayed
          // measured-speed reference would brake the accelerating knee.
          out.knee_kp_scale[side]=0.F;
          thrust_finished_[side]=thrust_finished_[side]||
              in.position_rad[k]>=config_.extension_knee_rad;
          out.velocity_rad_s[k]=thrust_finished_[side]?0.F:config_.max_joint_speed_rad_s;
          if (thrust_finished_[side]) {
            // Release is one-way within this jump, including contact flicker.
            out.feedforward_nm[h]=out.feedforward_nm[k]=0.F;
          }
        }
      }
    }
    if (phase_==Phase::Thrust || phase_==Phase::Flight || phase_==Phase::Land) {
      // Both hip axes are +Y. Positive child torque produces negative body
      // reaction torque, so this sign opposes body pitch and angular velocity.
      const float hip_torque = std::clamp(.5F * (config_.body_pitch_kp*in.pitch_rad +
          config_.body_pitch_kd*in.pitch_rate_rad_s),
          -config_.hip_attitude_torque_limit_nm, config_.hip_attitude_torque_limit_nm);
      for (auto h : {0,3}) out.feedforward_nm[h] = std::clamp(
          out.feedforward_nm[h]+hip_torque,
          -config_.max_leg_torque_nm, config_.max_leg_torque_nm);
    }
  }
  last_velocity_=out.velocity_rad_s;
  out.takeoff_com_z_m=takeoff_com_;out.apex_com_z_m=apex_com_;
  out.plan_elapsed_s=plan_time_s_;
  out.apex_gain_m=airborne_?std::max(0.,apex_com_-takeoff_com_):0;
  return out;
}
}
