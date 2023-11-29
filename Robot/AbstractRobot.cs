// Copyright 2020 Rensselaer Polytechnic Institute
//                Wason Technology, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections.Generic;
using System.Text;
using RobotRaconteur;
using com.robotraconteur.robotics.robot;
using System.IO;
using System.Linq;
using com.robotraconteur.robotics.joints;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using com.robotraconteur.geometry;
using com.robotraconteur.action;
using com.robotraconteur.robotics.trajectory;
using Rox = GeneralRoboticsToolbox;
using com.robotraconteur.eventlog;
using com.robotraconteur.robotics.tool;
using com.robotraconteur.robotics.payload;
using com.robotraconteur.device;
using com.robotraconteur.device.isoch;
using System.Security.Cryptography.X509Certificates;
using MathNet.Numerics.Integration;
using RobotRaconteur.Companion.Converters;
using com.robotraconteur.datetime;
using RobotRaconteur.Companion.Util;

namespace RobotRaconteur.Companion.Robot
{
    public abstract class AbstractRobot : Robot_default_impl, async_Robot, IDisposable, IRRServiceObject
    {        
        protected internal int _joint_count;
        protected internal string[] _joint_names;
        protected internal double _jog_joint_limit = Math.PI * (10000.0 / 180.0);
        protected internal double _trajectory_error_tol = Math.PI * (5.0 / 180.0);
                
        protected internal RobotCommandMode _command_mode = RobotCommandMode.halt;
        protected RobotOperationalMode _operational_mode = RobotOperationalMode.manual_reduced_speed;
        protected RobotControllerState _controller_state = RobotControllerState.undefined;

        protected double[] _joint_position = new double[0];
        protected double[] _joint_velocity = new double[0];
        protected double[] _joint_effort = new double[0];

        protected double[] _position_command = null;
        protected double[] _velocity_command = null;

        protected Pose[] _endpoint_pose;
        protected SpatialVelocity[] _endpoint_vel;

        protected internal bool _homed = false;
        protected internal bool _ready = false;
        protected internal bool _enabled = false;
        protected internal bool _stopped = false;
        protected internal bool _error = false;
        protected byte _estop_source = 0;

        protected bool _communication_failure = true;
        protected long _communication_timeout = 250; // milliseconds


        protected Stopwatch _stopwatch;
        protected TimeSpec2 _stopwatch_epoch;

        protected com.robotraconteur.uuid.UUID _robot_uuid;
        protected com.robotraconteur.robotics.robot.RobotInfo _robot_info;

        protected double _speed_ratio = 1;

        protected bool _uses_homing = false;

        protected bool _has_position_command = false;

        protected bool _has_velocity_command = false;

        protected bool _has_jog_command = true;

        protected uint _robot_caps;

        protected Rox.Robot[] _rox_robots;

        protected ToolInfo[] _current_tool = null;

        protected PayloadInfo[] _current_payload = null;

        protected BroadcastDownsampler _broadcast_downsampler;
        
        public AbstractRobot(com.robotraconteur.robotics.robot.RobotInfo robot_info, int default_joint_count)
        {            
            this._robot_info = robot_info;
            if (robot_info.joint_info != null)
            {
                var j_names = new List<string>();
                foreach (var j_info in robot_info.joint_info)
                {
                    j_names.Add(j_info.joint_identifier.name);
                }
                _joint_names = j_names.ToArray();
            }
            else
            {
                if (default_joint_count <= 0)
                {
                    throw new ArgumentException("Joints must be specified in RobotInfo structure");
                }
                _joint_names = Enumerable.Range(0, default_joint_count).Select(x => $"joint_{x}").ToArray();
            }

            _joint_count = _joint_names.Length;

            _robot_uuid = robot_info.device_info.device.uuid;

            _robot_caps = robot_info.robot_capabilities;

            if ((_robot_caps & (uint)RobotCapabilities.homing_command) != 0)
            {
                _uses_homing = true;
            }

            if ((_robot_caps & (uint)RobotCapabilities.position_command) != 0)
            {
                _has_position_command = true;
            }

            if ((_robot_caps & (uint)RobotCapabilities.velocity_command) != 0)
            {
                _has_velocity_command = true;
            }

            try
            {
                _rox_robots = new Rox.Robot[robot_info.chains.Count];
                for (int i = 0; i < robot_info.chains.Count; i++)
                {
                    _rox_robots[i] = RobotInfoConverter.ToToolboxRobot(robot_info, i);
                }
            }
            catch (Exception e)
            {
                throw new ArgumentException("invalid robot_info, could not populate GeneralRoboticsToolbox.Robot", e);
            }

            _current_tool = new ToolInfo[robot_info.chains.Count];
            _current_payload = new PayloadInfo[robot_info.chains.Count];

            for (int i=0; i<robot_info.chains.Count; i++)
            {
                if (robot_info.chains[i].current_tool != null)
                {
                    _current_tool[i] = robot_info.chains[i].current_tool;
                }

                if (robot_info.chains[i].current_payload != null)
                {
                    _current_payload[i] = robot_info.chains[i].current_payload;
                }
            }



            for (int i = 0; i < _joint_count; i++)
            {
                var limits = robot_info.joint_info[i].joint_limits;
                if (limits.velocity <= 0)
                {
                    throw new ArgumentException($"Invalid joint velocity for joint {i}");
                }
                if (limits.reduced_velocity <= 0)
                {
                    limits.reduced_velocity = limits.velocity;
                }

                if (limits.acceleration <= 0)
                {
                    throw new ArgumentException($"Invalid joint acceleration for joint {i}");
                }
                if (limits.reduced_acceleration <= 0)
                {
                    limits.reduced_acceleration = limits.acceleration;
                }
            }
        }

        protected bool _keep_going = false;

        public virtual void _start_robot()
        {
            if (!Stopwatch.IsHighResolution)
            {
                Debug.WriteLine("warning: not using high resolution timer");
            }
            _stopwatch = Stopwatch.StartNew();
            _stopwatch_epoch = DateTimeUtil.TimeSpec2Now(RobotRaconteurNode.s);
                        
            _keep_going = true;
            _loop_thread = new Thread(_loop_thread_func);
            _loop_thread.Start();
        }

        protected virtual void _stop_robot()
        {
            _keep_going = false;
            _loop_thread.Join();
        }

        protected Thread _loop_thread;

        protected int _update_period = 10;
        protected virtual void _loop_thread_func()
        { 
            // Use a spin wait loop to get higher timing accurancy
            SpinWait spin_wait = new SpinWait();

            long next_wait = _stopwatch.ElapsedMilliseconds;

            long now = next_wait;

            while (_keep_going)
            {
                _run_timestep(now);

                now = _stopwatch.ElapsedMilliseconds;
                
                do
                {
                    next_wait += _update_period;
                }
                while (next_wait <= now);

                spin_wait.Reset();
                while ((now = _stopwatch.ElapsedMilliseconds) < next_wait)
                {
                    spin_wait.SpinOnce();
                }
            }
        }

        protected long _last_robot_state;
        protected long _last_joint_state;
        protected long _last_endpoint_state;

        public long _now => _stopwatch.ElapsedMilliseconds;

        
        public virtual void Dispose()
        {
            _keep_going = false;
            
        }

        
        protected internal ulong _state_seqno = 0;

        protected internal virtual void _run_timestep(long now)
        {
            bool res;

            double[] joint_pos_cmd = null;
            double[] joint_vel_cmd = null;

            RobotState rr_robot_state;
            AdvancedRobotState rr_advanced_robot_state;
            RobotStateSensorData rr_state_sensor_data;
            BroadcastDownsamplerStep downsampler_step;
            lock (this)
            {
                downsampler_step = null;
                if (_broadcast_downsampler != null)
                {
                    downsampler_step = new BroadcastDownsamplerStep(_broadcast_downsampler);
                }

                _state_seqno++;

                res = _verify_communication(now);
                res = res && _verify_robot_state(now);
                res = res && _fill_robot_command(now, out joint_pos_cmd, out joint_vel_cmd);

                _fill_states(now, out rr_robot_state, out rr_advanced_robot_state, out rr_state_sensor_data);

            }

            if (!res)
            {
                //_send_disable();
            }
            else
            {
                _send_robot_command(now, joint_pos_cmd, joint_vel_cmd);
            }

            using (downsampler_step)
            {
                _send_states(now, rr_robot_state, rr_advanced_robot_state, rr_state_sensor_data);                
            }
        }

        protected internal virtual ulong _fill_state_flags(long now)
        {
            ulong f = 0;

            if (_communication_failure)
            {
                f |= (ulong)RobotStateFlags.communication_failure;
                return f;
            }

            if (_error)
            {
                f |= (ulong)RobotStateFlags.error;
            }

            if (_stopped)
            {
                f |= (ulong)RobotStateFlags.estop;

                switch(_estop_source)
                {
                    case 0:
                        break;
                    case 1:
                        f |= (ulong)RobotStateFlags.estop_button1;
                        break;
                    case 2:
                        f |= (ulong)RobotStateFlags.estop_other;
                        break;
                    case 3:
                        f |= (ulong)RobotStateFlags.estop_fault;
                        break;
                    case 4:
                        f |= (ulong)RobotStateFlags.estop_internal;
                        break;
                    default:
                        break;
                }
            }

            if (_enabled)
            {
                f |= (ulong)RobotStateFlags.enabled;
            }

            if (_ready)
            {
                f |= (ulong)RobotStateFlags.ready;
            }

            if (_uses_homing)
            {
                if (_homed)
                {
                    f |= (ulong)RobotStateFlags.homed;
                }
                else
                {
                    f |= (ulong)RobotStateFlags.homing_required;
                }
            }

            if (_wire_position_command_sent)
            {
                f |= (ulong)RobotStateFlags.valid_position_command;
            }

            if (_wire_velocity_command_sent)
            {
                f |= (ulong)RobotStateFlags.valid_velocity_command;
            }

            if (_trajectory_valid)
            {
                f |= (ulong)RobotStateFlags.trajectory_running;
            }

            return f;

        }

        protected virtual Pose _calc_endpoint_pose(int chain)
        {
            // CALL LOCKED!
            if (_current_tool[chain] == null)
            {
                return _endpoint_pose[chain];
            }
            var endpoint_transform = GeometryConverter.ToTransform(_endpoint_pose[chain]);
            var tool_transform = GeometryConverter.ToTransform(_current_tool[chain].tcp);
            var res = endpoint_transform * tool_transform;
            return GeometryConverter.ToPose(res);            
        }

        protected virtual Pose[] _calc_endpoint_poses()
        {
            if (_endpoint_pose == null)
            {
                return new Pose[0];
            }

            var o = new Pose[_endpoint_pose.Length];
            for (int i=0; i<o.Length; i++)
            {
                o[i] = _calc_endpoint_pose(i);
            }
            return o;
        }

        protected virtual SpatialVelocity _calc_endpoint_vel(int chain)
        {
            // CALL LOCKED!
            if (_current_tool[chain] == null)
            {
                return _endpoint_vel[chain];
            }
            var endpoint_vel_lin = GeometryConverter.ToVector(_endpoint_vel[chain].linear);
            var endpoint_vel_ang = GeometryConverter.ToVector(_endpoint_vel[chain].angular);
            var current_tool_p = GeometryConverter.ToVector(_current_tool[chain].tcp.translation);            
            
            var endpoint_transform = GeometryConverter.ToTransform(_endpoint_pose[chain]);
            
            var o = new SpatialVelocity();
            o.linear = GeometryConverter.ToVector3(endpoint_vel_lin + Rox.Functions.Cross(endpoint_vel_ang, endpoint_transform.R * current_tool_p));
            o.angular = _endpoint_vel[chain].angular;

            return o;
        }

        protected virtual SpatialVelocity[] _calc_endpoint_vels()
        {
            if (_endpoint_vel == null)
            {
                return new SpatialVelocity[0];
            }

            var o = new SpatialVelocity[_endpoint_pose.Length];
            for (int i = 0; i < o.Length; i++)
            {
                o[i] = _calc_endpoint_vel(i);
            }
            return o;
        }


        protected internal virtual void _fill_states(long now, out RobotState rr_robot_state, out AdvancedRobotState rr_advanced_robot_state, out RobotStateSensorData rr_state_sensor_data)
        {
            var ts = DateTimeUtil.TimeSpec3Now(RobotRaconteurNode.s);

            var rob_state = new RobotState();
            rob_state.ts = ts;
            rob_state.seqno = _state_seqno;
            rob_state.command_mode = _command_mode;
            rob_state.operational_mode = _operational_mode;
            rob_state.controller_state = _controller_state;

            var flags = _fill_state_flags(now);

            rob_state.robot_state_flags = flags;

            rob_state.joint_position = (double[])_joint_position.Clone();
            rob_state.joint_velocity = (double[])_joint_velocity.Clone();
            rob_state.joint_effort = (double[])_joint_effort.Clone();
            rob_state.joint_position_command = _position_command ??new double[0];
            rob_state.joint_velocity_command = _velocity_command ?? new double[0];
            rob_state.kin_chain_tcp = _calc_endpoint_poses();
            rob_state.kin_chain_tcp_vel = _calc_endpoint_vels();
            rob_state.trajectory_running = _trajectory_valid;

            var a_rob_state = new AdvancedRobotState();
            a_rob_state.ts = ts;
            a_rob_state.seqno = rob_state.seqno;
            a_rob_state.command_mode = rob_state.command_mode;
            a_rob_state.operational_mode = rob_state.operational_mode;
            a_rob_state.controller_state = rob_state.controller_state;
            a_rob_state.joint_position = rob_state.joint_position;
            a_rob_state.joint_velocity = rob_state.joint_velocity;
            a_rob_state.joint_effort = rob_state.joint_effort;
            a_rob_state.joint_position_command = rob_state.joint_position_command;
            a_rob_state.joint_velocity_command = rob_state.joint_velocity_command;
            a_rob_state.kin_chain_tcp = rob_state.kin_chain_tcp;
            a_rob_state.trajectory_running = rob_state.trajectory_running;
            a_rob_state.joint_position_units = Enumerable.Repeat<byte>((byte)JointPositionUnits.radian, 7).ToArray();
            a_rob_state.joint_effort_units = Enumerable.Repeat<byte>((byte)JointEffortUnits.newton_meter, 7).ToArray();
            a_rob_state.trajectory_running = _trajectory_valid;
            a_rob_state.trajectory_time = _trajectory_current_time;
            a_rob_state.trajectory_max_time = _trajectory_max_time;
            a_rob_state.trajectory_current_waypoint = _trajectory_waypoint;
                                    
            var sensor_data_header = SensorDataUtil.FillSensorDataHeader(RobotRaconteurNode.s,_robot_info?.device_info, _state_seqno);
            
            var sensor_data = new RobotStateSensorData();
            sensor_data.data_header = sensor_data_header;
            sensor_data.robot_state = a_rob_state;

            rr_robot_state = rob_state;
            rr_advanced_robot_state = a_rob_state;
            rr_state_sensor_data = sensor_data;
        }

        protected virtual void _send_states(long now, RobotState rr_robot_state, AdvancedRobotState rr_advanced_robot_state, RobotStateSensorData rr_state_sensor_data)
        {
            if (rrvar_robot_state != null)
            {
                rrvar_robot_state.OutValue = rr_robot_state;
            }

            if (rrvar_advanced_robot_state != null)
            {
                rrvar_advanced_robot_state.OutValue = rr_advanced_robot_state;
            }

            rrvar_robot_state_sensor_data?.AsyncSendPacket(rr_state_sensor_data).ContinueWith(t => { var ignore = t.Exception; }, System.Threading.Tasks.TaskContinuationOptions.OnlyOnFaulted);                        
            
            if (rrvar_device_clock_now != null)
            {
                rrvar_device_clock_now.OutValue = DateTimeUtil.FillDeviceTime(RobotRaconteurNode.s, robot_info?.device_info, _state_seqno);
            }
        }

        protected internal abstract Task _send_disable();
        
        public virtual async Task async_disable(int timeout = -1)
        {
            await _send_disable();
        }

        public virtual async Task async_enable(int timeout = -1)
        {
            await _send_enable();
        }

        protected internal abstract Task _send_enable();

        public virtual async Task async_reset_errors(int timeout = -1)
        {
            await _send_reset_errors();
        }

        protected internal abstract Task _send_reset_errors();


        public virtual Task async_halt(int timeout = -1)
        {
            if (_command_mode == RobotCommandMode.invalid_state)
            {
                return Task.FromResult(0);
            }
            _command_mode = RobotCommandMode.halt;

            return Task.FromResult(0);
        }
        
        protected virtual bool _verify_communication(long now)
        {   
            if (now - _last_joint_state > _communication_timeout
                || now - _last_robot_state > _communication_timeout
                || now - _last_endpoint_state > _communication_timeout)                
            {
                _communication_failure = true;

                _command_mode = RobotCommandMode.invalid_state;
                _operational_mode = RobotOperationalMode.undefined;
                _controller_state = RobotControllerState.undefined;

                _joint_position = new double[0];
                _joint_velocity = new double[0];
                _joint_effort = new double[0];

                _endpoint_pose = null;
                _endpoint_vel = null;

                //_send_disable();

                return false;
            }

            _operational_mode = RobotOperationalMode.cobot;
                        
            _communication_failure = false;
            return true;
        }

        protected virtual bool _verify_robot_state(long now)
        {
            if (_command_mode == RobotCommandMode.homing)
            {
                if (_enabled && !_error && ! _communication_failure)
                {
                    _controller_state = RobotControllerState.motor_off;
                    return true;
                }
            }

            if (!_ready || _communication_failure)
            {
                if (_stopped)
                {
                    _controller_state = RobotControllerState.emergency_stop;
                }
                else if (_error)
                {
                    _controller_state = RobotControllerState.guard_stop;
                }
                else
                {
                    _controller_state = RobotControllerState.motor_off;
                }

                _command_mode = RobotCommandMode.invalid_state;                
                return false;
            }
            
            if (!_enabled)
            {
                _controller_state = RobotControllerState.motor_off;
                _command_mode = RobotCommandMode.invalid_state;
                return false;
            }


            if (_command_mode == RobotCommandMode.invalid_state)
            {
                _command_mode = RobotCommandMode.halt;
            }

            _controller_state = RobotControllerState.motor_on;

            return true;
        }

        protected bool _wire_position_command_sent;
        protected bool _wire_velocity_command_sent;

        protected ulong _wire_position_command_last_seqno = 0;
        protected ulong _wire_velocity_command_last_seqno = 0;

        protected uint _wire_position_command_last_ep = 0;
        protected uint _wire_velocity_command_last_ep = 0;

        protected bool _trajectory_valid = false;
        protected double _trajectory_current_time;
        protected double _trajectory_max_time;
        protected uint _trajectory_waypoint;

        protected internal virtual bool _fill_robot_command(long now, out double[] joint_pos_cmd, out double[] joint_vel_cmd)
        {
            joint_pos_cmd = null;
            joint_vel_cmd = null;

            _wire_position_command_sent = false;
            _wire_velocity_command_sent = false;

            _trajectory_valid = false;
            _trajectory_current_time = 0;
            _trajectory_max_time = 0;
            _trajectory_waypoint = 0;

            if (_command_mode != RobotCommandMode.trajectory)
            {
                if (_active_trajectory != null)
                {
                    _active_trajectory.invalid_mode();
                    _active_trajectory = null;
                }
                
                if (_queued_trajectories.Count > 0)
                {
                    foreach (var t in _queued_trajectories)
                    {
                        t.invalid_mode();
                    }

                    _queued_trajectories.Clear();
                }
            }

            if (_command_mode != RobotCommandMode.velocity_command)
            {
                //_velocity_command = null;
            }

            switch (_command_mode)
            {
                case RobotCommandMode.jog:
                    {
                        
                        if (_jog_trajectory_generator != null)
                        {
                            
                            double jog_time = (now - _jog_start_time)/1000.0;

                            if (jog_time > _jog_trajectory_generator.T_Final)
                            {
                                if (_jog_completion_source != null)
                                {
                                    _jog_completion_source.TrySetResult(0);
                                    _jog_completion_source = null;
                                }
                                _jog_trajectory_generator = null;
                                return false;
                            }

                            if (!_jog_trajectory_generator.GetCommand(jog_time, out var jog_command))
                            {                                
                                return false;
                            }
                            joint_pos_cmd = jog_command.command_position;
                            return true;
                        }
                        else
                        {
                            if (_jog_completion_source != null)
                            {
                                _jog_completion_source.TrySetResult(0);
                                _jog_completion_source = null;
                            }
                        }
                                                
                        return true;
                    }
                case RobotCommandMode.position_command:
                    {
                        RobotJointCommand pos_cmd = default;
                        TimeSpec ts;
                        uint ep = default;
                        if (!(rrvar_position_command?.TryGetInValue(out pos_cmd, out ts, out ep) ?? false))
                        {                            
                            return true;
                        }

                        if (_wire_position_command_last_ep != ep)
                        {
                            _wire_position_command_last_ep = ep;
                            _wire_position_command_last_seqno = 0;
                        }

                        if (pos_cmd == null 
                            || pos_cmd.seqno < _wire_position_command_last_seqno
                            || Math.Abs((long)pos_cmd.state_seqno - (long)_state_seqno) > 10
                            || pos_cmd.command.Length != _joint_count
                            || pos_cmd.units.Length != 0 && pos_cmd.units.Length != _joint_count)
                        {
                            
                            return true;
                        }

                        double[] pos_cmd_j;
                        if (pos_cmd.units.Length == 0)
                        {
                            pos_cmd_j = pos_cmd.command;
                        }
                        else
                        {
                            pos_cmd_j = new double[_joint_count];
                            for (int i = 0; i < _joint_count; i++)
                            {
                                switch ((JointPositionUnits)pos_cmd.units[i])
                                {
                                    case JointPositionUnits.implicit_:
                                    case JointPositionUnits.radian:
                                        pos_cmd_j[i] = pos_cmd.command[i];
                                        break;
                                    case JointPositionUnits.degree:
                                        pos_cmd_j[i] = pos_cmd.command[i] * (Math.PI / 180.0);
                                        break;
                                    case JointPositionUnits.ticks_rot:
                                        pos_cmd_j[i] = (pos_cmd.command[i] / (double)(2 ^ 20)) * (Math.PI * 2.0);
                                        break;
                                    case JointPositionUnits.nanoticks_rot:
                                        pos_cmd_j[i] = (pos_cmd.command[i] / (double)(2 ^ 20) * 1e9) * (Math.PI * 2.0);
                                        break;
                                    default:
                                        {
                                            // Invalid units!
                                            
                                            return true;
                                        }
                                }
                            }
                        }                        

                        _wire_position_command_last_seqno = pos_cmd.seqno;
                        
                        joint_pos_cmd = pos_cmd_j;
                        
                        _wire_position_command_sent = true;

                        return true;
                    }
                case RobotCommandMode.velocity_command:
                    {
                        RobotJointCommand vel_cmd = default;
                        TimeSpec ts;
                        uint ep = default;
                        if (!(rrvar_velocity_command?.TryGetInValue(out vel_cmd, out ts, out ep) ?? false))
                        {                            
                            return true;
                        }

                        if (_wire_velocity_command_last_ep != ep)
                        {
                            _wire_velocity_command_last_ep = ep;
                            _wire_velocity_command_last_seqno = 0;
                        }

                        if (vel_cmd == null
                            || vel_cmd.seqno < _wire_velocity_command_last_seqno
                            || Math.Abs((long)vel_cmd.state_seqno - (long)_state_seqno) > 50
                            || vel_cmd.command.Length != _joint_count
                            || vel_cmd.units.Length != 0 && vel_cmd.units.Length != _joint_count)
                        {                            
                            return true;
                        }

                        double[] vel_cmd_j;
                        if (vel_cmd.units.Length == 0)
                        {
                            vel_cmd_j = vel_cmd.command;
                        }
                        else
                        {
                            vel_cmd_j = new double[_joint_count];
                            for (int i = 0; i < _joint_count; i++)
                            {
                                switch ((JointVelocityUnits)vel_cmd.units[i])
                                {
                                    case JointVelocityUnits.implicit_:
                                    case JointVelocityUnits.radian_second:
                                        vel_cmd_j[i] = vel_cmd.command[i];
                                        break;
                                    case JointVelocityUnits.degree_second:
                                        vel_cmd_j[i] = vel_cmd.command[i] * (Math.PI / 180.0);
                                        break;
                                    case JointVelocityUnits.ticks_rot_second:
                                        vel_cmd_j[i] = (vel_cmd.command[i] / (double)(2 ^ 20)) * (Math.PI * 2.0);
                                        break;
                                    case JointVelocityUnits.nanoticks_rot_second:
                                        vel_cmd_j[i] = (vel_cmd.command[i] / (double)(2 ^ 20) * 1e9) * (Math.PI * 2.0);
                                        break;
                                    default:
                                        {
                                            // Invalid units!
                                            return true;
                                        }
                                }
                            }
                        }

                        _wire_velocity_command_last_seqno = vel_cmd.seqno;

                        if (_speed_ratio != 1.0)
                        {
                            for (int i=0; i<vel_cmd_j.Length; i++)
                            {
                                vel_cmd_j[i] = vel_cmd_j[i] * _speed_ratio;
                            }
                        }
                                                
                        joint_vel_cmd = vel_cmd_j;

                        _wire_velocity_command_sent = true;

                        return true;
                    }
                case RobotCommandMode.trajectory:
                    {
                        if (_active_trajectory != null)
                        {
                            bool send_traj_cmd;
                            var interp_res = _active_trajectory.get_setpoint(now, _joint_position, out var traj_pos, out var traj_vel, out var traj_t, out var traj_max_time, out var traj_waypoint);
                            switch (interp_res)
                            {
                                case TrajectoryTaskRes.ready:
                                    _trajectory_valid = true;
                                    send_traj_cmd = false;
                                    break;
                                case TrajectoryTaskRes.first_valid_setpoint:
                                case TrajectoryTaskRes.valid_setpoint:
                                    _trajectory_valid = true;
                                    send_traj_cmd = true;
                                    break;
                                case TrajectoryTaskRes.trajectory_complete:
                                    _trajectory_valid = true;
                                    send_traj_cmd = true;
                                    _active_trajectory = null;
                                    if (_queued_trajectories.Count >0)
                                    {
                                        _active_trajectory = _queued_trajectories[0];
                                        _queued_trajectories.RemoveAt(0);
                                    }
                                    break;
                                default:
                                    _trajectory_valid = false;
                                    send_traj_cmd = false;
                                    _active_trajectory = null;
                                    foreach (var w in _queued_trajectories)
                                    {
                                        w._cancelled_in_queue();
                                    }
                                    _queued_trajectories.Clear();
                                    break;
                            }

                            if (_trajectory_valid)
                            {
                                _trajectory_current_time = traj_t;
                                _trajectory_max_time = traj_max_time;
                                _trajectory_waypoint = (uint)traj_waypoint;
                            }

                            if (send_traj_cmd)
                            {
                                joint_pos_cmd = traj_pos;
                            }
                            else
                            {
                                joint_pos_cmd = null;
                            }
                        }
                        else
                        {
                            joint_pos_cmd = null;
                        }
                        return true;
                    }
                default:
                    {
                        joint_pos_cmd = null;
                        return true;
                    }
            }
        }

        protected abstract void _send_robot_command(long now, double[] joint_pos_cmd, double[] joint_vel_cmd);
       

        public virtual Task<RobotCommandMode> async_get_command_mode(int timeout = -1)
        {
            lock (this)
            {
                return Task.FromResult(_command_mode);
            }
        }
        
        public virtual Task async_set_command_mode(RobotCommandMode value, int timeout = -1)
        {
            lock (this)
            {
                if (_command_mode == RobotCommandMode.invalid_state && value == RobotCommandMode.homing)
                {
                    if (!_enabled || _communication_failure)
                    {
                        throw new InvalidOperationException("Cannot set homing command mode in current state");
                    }

                    _command_mode = RobotCommandMode.homing;
                    return Task.FromResult(0);
                }

                if (!_ready || _communication_failure)
                {
                    throw new InvalidOperationException("Cannot set robot command mode in current state");
                }

                if (_command_mode != RobotCommandMode.halt && value != RobotCommandMode.halt)
                {
                    throw new InvalidOperationException("Must switch to \"halt\" before selecting new mode");
                }

                switch (value)
                {
                    case RobotCommandMode.jog:
                        {
                            if (!_has_jog_command)
                            {
                                throw new InvalidOperationException("Robot does not support jog command mode");
                            }
                            _jog_trajectory_generator = null;
                            _command_mode = RobotCommandMode.jog;
                            break;
                        }
                    case RobotCommandMode.halt:
                        {
                            _command_mode = value;
                            break;
                        }
                    case RobotCommandMode.homing:
                        {
                            if (!_uses_homing)
                            {
                                throw new InvalidOperationException("Robot does not support homing mode");
                            }
                            _command_mode = value;
                        }
                        break;
                    case RobotCommandMode.position_command:
                        {
                            if (!_has_position_command)
                            {
                                throw new InvalidOperationException("Robot does not support position command mode");
                            }
                            _command_mode = value;
                            break;
                        }
                    case RobotCommandMode.velocity_command:
                        {
                            if (!_has_velocity_command)
                            {
                                throw new InvalidOperationException("Robot does not support velocity command mode");
                            }
                            _command_mode = value;
                            break;
                        }
                    case RobotCommandMode.trajectory:
                        _command_mode = value;
                        break;
                    default:
                        throw new ArgumentException("Invalid command mode specified");
                }
            }

            return Task.FromResult(0);
        }


        protected double _jog_start_time;
        protected JointTrajectoryGenerator _jog_trajectory_generator;
        protected TaskCompletionSource<int> _jog_completion_source;


        public virtual Task async_jog_freespace(double[] joint_position, double[] max_velocity, bool wait, int timeout = -1)
        {
            lock(this)
            {
                if (_command_mode != RobotCommandMode.jog)
                {
                    throw new InvalidOperationException("Robot not in jog mode");
                }

                if (!_ready)
                {
                    throw new OperationAbortedException("Robot not ready");
                }
                
                if (joint_position.Length != _joint_count)
                {
                    throw new ArgumentException($"joint_position array must have {_joint_count} elements");
                }

                if (max_velocity.Length != _joint_count)
                {
                    throw new ArgumentException($"max_velocity array must have {_joint_count} elements");
                }

                for (int i=0; i<_joint_count; i++)
                {
                    if (Math.Abs(_joint_position[i] - joint_position[i]) > _jog_joint_limit)
                    {
                        throw new ArgumentException("Position command must be within 15 degrees from current");
                    }

                    if (max_velocity[i] < 0)
                    {
                        throw new ArgumentException("max_vel must be greater than zero");
                    }
                }


                if (_jog_completion_source != null)
                {
                    _jog_completion_source.TrySetException(new OperationAbortedException("Operation interrupted by new jog command"));
                    _jog_completion_source = null;
                }

                long now = _stopwatch.ElapsedMilliseconds;

                if (_jog_trajectory_generator == null)
                {
                    var limits = new JointTrajectoryLimits();
                    switch (_operational_mode)
                    {
                        case RobotOperationalMode.manual_reduced_speed:
                            limits.a_max = _robot_info.joint_info.Select(x => x.joint_limits.reduced_acceleration).ToArray();
                            limits.v_max = _robot_info.joint_info.Select(x => x.joint_limits.reduced_velocity).ToArray();
                            break;
                        case RobotOperationalMode.manual_full_speed:
                        case RobotOperationalMode.cobot:
                            limits.a_max = _robot_info.joint_info.Select(x => x.joint_limits.acceleration).ToArray();
                            limits.v_max = _robot_info.joint_info.Select(x => x.joint_limits.velocity).ToArray();
                            break;
                        default:
                            throw new InvalidOperationException("Invalid operation mode for robot jog");
                    }
                
                    limits.x_min = _robot_info.joint_info.Select(x => x.joint_limits.lower).ToArray();
                    limits.x_max = _robot_info.joint_info.Select(x => x.joint_limits.upper).ToArray();

                    for (int i = 0; i < _joint_count; i++)
                    {
                        if (Math.Abs(max_velocity[i]) > limits.v_max[i])
                        {
                            throw new ArgumentException($"max_velocity[{i}] is greater than joint limits ({limits.v_max[i]})");
                        }
                    }

                    _jog_trajectory_generator = new TrapezoidalJointTrajectoryGenerator((uint)_joint_count, ref limits);

                    var new_req = new JointTrajectoryPositionRequest();
                    new_req.current_position = _position_command ?? (double[])_joint_position.Clone();
                    new_req.current_velocity = _velocity_command ?? new double[_joint_count];
                    new_req.desired_position = joint_position;
                    new_req.desired_velocity = new double[_joint_count];
                    new_req.max_velocity = max_velocity;
                    new_req.speed_ratio = _speed_ratio;

                    _jog_trajectory_generator.UpdateDesiredPosition(ref new_req);
                    _jog_start_time = now;
                }
                else
                {
                    double jog_trajectory_t = (now - _jog_start_time)/1000.0;
                    if (!_jog_trajectory_generator.GetCommand(jog_trajectory_t, out var cmd))
                    {
                        throw new InvalidOperationException("Cannot update jog command");
                    }

                    var new_req = new JointTrajectoryPositionRequest();
                    new_req.current_position = cmd.command_position;
                    new_req.current_velocity = cmd.command_velocity;
                    new_req.desired_position = joint_position;
                    new_req.desired_velocity = new double[_joint_count];
                    new_req.max_velocity = max_velocity;
                    new_req.speed_ratio = _speed_ratio;

                    _jog_trajectory_generator.UpdateDesiredPosition(ref new_req);
                    _jog_start_time = now;
                }

                if (!wait)
                {
                    _jog_completion_source = null;
                    return Task.FromResult(0);
                }
                else
                {
                    _jog_completion_source = new TaskCompletionSource<int>();
                    return _jog_completion_source.Task;
                }
            }
        }

        public virtual Task async_jog_joint(double[] joint_velocity, double timeout, bool wait, int rr_timeout = -1)
        {
            lock (this)
            {
                if (_command_mode != RobotCommandMode.jog)
                {
                    throw new InvalidOperationException("Robot not in jog mode");
                }

                if (!_ready)
                {
                    throw new OperationAbortedException("Robot not ready");
                }

                if (joint_velocity.Length != _joint_count)
                {
                    throw new ArgumentException($"joint_velocity array must have {_joint_count} elements");
                }

                if (timeout <= 0)
                {
                    throw new ArgumentException("Invalid jog timeout specified");
                }

                for (int i=0; i< _joint_count; i++)
                {
                    if (Math.Abs(joint_velocity[i]) > _robot_info.joint_info[i].joint_limits.reduced_velocity)
                    {
                        throw new ArgumentException("Joint velocity exceeds joint limits");
                    }
                }

                if (_jog_completion_source != null)
                {
                    _jog_completion_source.TrySetException(new OperationAbortedException("Operation interrupted by new jog command"));
                    _jog_completion_source = null;
                }

                long now = _stopwatch.ElapsedMilliseconds;

                if (_jog_trajectory_generator == null)
                {
                    var limits = new JointTrajectoryLimits();
                    switch (_operational_mode)
                    {
                        case RobotOperationalMode.manual_reduced_speed:
                            limits.a_max = _robot_info.joint_info.Select(x => x.joint_limits.reduced_acceleration).ToArray();
                            limits.v_max = _robot_info.joint_info.Select(x => x.joint_limits.reduced_velocity).ToArray();
                            break;
                        case RobotOperationalMode.manual_full_speed:
                        case RobotOperationalMode.cobot:
                            limits.a_max = _robot_info.joint_info.Select(x => x.joint_limits.acceleration).ToArray();
                            limits.v_max = _robot_info.joint_info.Select(x => x.joint_limits.velocity).ToArray();
                            break;
                        default:
                            throw new InvalidOperationException("Invalid operation mode for robot jog");
                    }
                    limits.x_min = _robot_info.joint_info.Select(x => x.joint_limits.lower).ToArray();
                    limits.x_max = _robot_info.joint_info.Select(x => x.joint_limits.upper).ToArray();

                    _jog_trajectory_generator = new TrapezoidalJointTrajectoryGenerator((uint)_joint_count, ref limits);

                    var new_req = new JointTrajectoryVelocityRequest();
                    new_req.current_position = _position_command ?? _joint_position;
                    new_req.current_velocity = _velocity_command ?? new double[_joint_count];
                    new_req.desired_velocity = joint_velocity;
                    new_req.speed_ratio = _speed_ratio;
                    new_req.timeout = timeout;

                    _jog_trajectory_generator.UpdateDesiredVelocity(ref new_req);
                    _jog_start_time = now;
                }
                else
                {
                    double jog_trajectory_t = (now - _jog_start_time) / 1000.0;
                    if (!_jog_trajectory_generator.GetCommand(jog_trajectory_t, out var cmd))
                    {
                        throw new InvalidOperationException("Cannot update jog command");
                    }

                    var new_req = new JointTrajectoryVelocityRequest();
                    new_req.current_position = cmd.command_position;
                    new_req.current_velocity = cmd.command_velocity;
                    new_req.desired_velocity = joint_velocity;
                    new_req.timeout = timeout;
                    new_req.speed_ratio = _speed_ratio;

                    _jog_trajectory_generator.UpdateDesiredVelocity(ref new_req);
                    _jog_start_time = now;
                }

                if (!wait)
                {
                    _jog_completion_source = null;
                    return Task.FromResult(0);
                }
                else
                {
                    _jog_completion_source = new TaskCompletionSource<int>();
                    return _jog_completion_source.Task;
                }
            }
        }

        public virtual Task<RobotInfo> async_get_robot_info(int timeout = -1)
        {
            lock (this)
            {
                for (int i = 0; i < _robot_info.chains.Count; i++)
                {
                    _robot_info.chains[i].current_tool = _current_tool[i];
                    _robot_info.chains[i].current_payload = _current_payload[i];
                }
                return Task.FromResult(_robot_info);
            }
        }
        
        protected TrajectoryTask _active_trajectory;
        protected List<TrajectoryTask> _queued_trajectories = new List<TrajectoryTask>();
        public override Generator2<com.robotraconteur.robotics.trajectory.TrajectoryStatus> execute_trajectory(com.robotraconteur.robotics.trajectory.JointTrajectory trajectory)
        {
            
            uint owner_ep = ServerEndpoint.CurrentEndpoint;


            double[] current_joint_pos;
            double speed_ratio;
            lock(this)
            {
                if (_command_mode != RobotCommandMode.trajectory)
                {
                    throw new InvalidOperationException("Robot must be in trajectory mode to execute trajectory");
                }

                speed_ratio = _speed_ratio;
                current_joint_pos = _joint_position;
            }
            
            
            var interp = new JointTrajectoryInterpolator(_robot_info);
            interp.LoadTrajectory(trajectory, speed_ratio);
                       
            interp.Intepolate(0, out var joint_pos1, out var current_waypoint1);

            for (int i = 0; i < current_joint_pos.Length; i++)
            {
                if (Math.Abs(current_joint_pos[i] - joint_pos1[i]) > _trajectory_error_tol)
                {
                    throw new ArgumentException("Starting waypoint too far from current joint positions");
                }
            }

            lock (this)
            {
                if (_command_mode != RobotCommandMode.trajectory)
                {
                    throw new InvalidOperationException("Robot must be in trajectory mode to execute trajectory");
                }

                TrajectoryTask traj_task;

                if (_active_trajectory == null)
                {
                    traj_task = new TrajectoryTask(this, interp, false, owner_ep);
                    _active_trajectory = traj_task;
                }
                else
                {
                    traj_task = new TrajectoryTask(this, interp, true, owner_ep);
                    _queued_trajectories.Add(traj_task);
                }

                return traj_task;
            }

            throw new NotImplementedException("Trajectory passed checks");

        }

        protected internal virtual void _cancel_trajectory(TrajectoryTask trajectory)
        {
            lock (this)
            {
                if (trajectory.Equals(_active_trajectory))
                {
                    _active_trajectory = null;
                    foreach (var t in _queued_trajectories)
                    {
                        t._cancelled_in_queue();
                    }
                    _queued_trajectories.Clear();
                }
                else
                {
                    int t_index = -1;
                    for (int i = 0; i<_queued_trajectories.Count; i++)
                    {
                        if (trajectory.Equals(_queued_trajectories[i]))
                        {
                            t_index = i;
                        }
                    }

                    for (int i = _queued_trajectories.Count - 1; i > t_index; i--)
                    {
                        _queued_trajectories[i]._cancelled_in_queue();
                        _queued_trajectories.RemoveAt(i);
                    }

                    _queued_trajectories.RemoveAt(t_index);
                }
            }
        }

        internal protected void _abort_trajectory(TrajectoryTask trajectory)
        {
            _command_mode = RobotCommandMode.halt;
        }
        
        public virtual Task<double> async_get_speed_ratio(int timeout = -1)
        {
            return Task.FromResult(_speed_ratio);
        }

        
        public virtual Task async_set_speed_ratio(double value, int timeout = -1)
        {
            if (value < 0.1 || value > 10)
            {
                throw new ArgumentException("Invalid speed_ratio");
            }

            _speed_ratio = value;
            return Task.FromResult(0);
        }

        public virtual Task<RobotOperationalMode> async_get_operational_mode(int rr_timeout = -1)
        {
            lock(this)
            {
                return Task.FromResult(_operational_mode);
            }
        }

        public virtual Task<RobotControllerState> async_get_controller_state(int rr_timeout = -1)
        {
            lock(this)
            {
                return Task.FromResult(_controller_state);
            }
        }

        public virtual Task<List<EventLogMessage>> async_get_current_errors(int rr_timeout = -1)
        {
            return Task.FromResult(new List<EventLogMessage>());
        }

        public virtual Task async_jog_cartesian(Dictionary<int, SpatialVelocity> velocity, double timeout, bool wait, int rr_timeout = -1)
        {
            throw new NotImplementedException();
        }

        public virtual Task<Generator2<TrajectoryStatus>> async_execute_trajectory(JointTrajectory trajectory, int rr_timeout = -1)
        {
            // NOTE: Not called because this is a generator function, execute_trajector is called instead
            throw new InvalidOperationException("Call execute_trajectory()");
        }

        public virtual Task<Generator2<ActionStatusCode>> async_home(int rr_timeout = -1)
        {
            throw new NotImplementedException();
        }

        public virtual Task<double[]> async_getf_signal(string signal_name, int rr_timeout = -1)
        {
            throw new NotImplementedException();
        }

        public virtual Task async_setf_signal(string signal_name, double[] value_, int rr_timeout = -1)
        {
            throw new NotImplementedException();
        }

        public virtual Task async_tool_attached(int chain, ToolInfo tool, int rr_timeout = -1)
        {
            if (tool == null)
            {
                throw new NullReferenceException("Tool cannot be null");
            }
            if (chain < 0 || !(chain < _current_tool.Length))
            {
                throw new ArgumentException($"Invalid kinematic chain {chain} for tool");
            }
            lock(this)
            {
                if (_current_tool[chain] != null)
                {
                    throw new ArgumentException($"Tool alread attached to kinematic chain {chain}");
                }

                _current_tool[chain] = tool;

                rrfire_tool_changed(chain, tool?.device_info?.device?.name ?? "");
                _state_seqno++;                
            }

            return Task.FromResult(0);
        }

        public virtual Task async_tool_detached(int chain, string tool_name, int rr_timeout = -1)
        {
            if (chain < 0 || !(chain < _current_tool.Length))
            {
                throw new ArgumentException($"Invalid kinematic chain {chain} for tool");
            }

            lock (this)
            {
                if (_current_tool[chain] == null)
                {
                    throw new ArgumentException($"Tool not attached to kinematic chain {chain}");
                }

                if (_current_payload[chain] != null)
                {
                    throw new ArgumentException($"Cannot remove tool while payload attached");
                }

                if (!string.IsNullOrEmpty(tool_name))
                {
                    if (_current_tool[chain]?.device_info?.device?.name != tool_name)
                    {
                        throw new ArgumentException($"Invalid tool name to detach from kinematic chain {chain}");
                    }
                }

                _current_tool[chain] = null;

                rrfire_tool_changed(chain, "");
                _state_seqno++;
            }

            return Task.FromResult(0);

        }

        public virtual Task async_payload_attached(int chain, PayloadInfo payload, Pose pose, int rr_timeout = -1)
        {
            if (payload == null)
            {
                throw new NullReferenceException("Tool cannot be null");
            }
            if (chain < 0 || !(chain < _current_tool.Length))
            {
                throw new ArgumentException($"Invalid kinematic chain {chain} for payload");
            }
            lock (this)
            {
                if (_current_tool[chain] == null)
                {
                    throw new ArgumentException($"No tool attached to kinematic chain {chain}, cannot attach payload");
                }

                if (_current_payload[chain] != null)
                {
                    throw new ArgumentException($"Payload alread attached to kinematic chain {chain}");
                }

                _current_payload[chain] = payload;

                rrfire_payload_changed(chain, payload?.device_info?.device?.name ?? "");
                _state_seqno++;
            }

            return Task.FromResult(0);
        }

        public virtual Task async_payload_detached(int chain, string payload_name, int rr_timeout = -1)
        {
            if (chain < 0 || !(chain < _current_tool.Length))
            {
                throw new ArgumentException($"Invalid kinematic chain {chain} for tool");
            }

            lock (this)
            {
                if (_current_payload[chain] == null)
                {
                    throw new ArgumentException($"Payload not attached to kinematic chain {chain}");
                }

                if (!string.IsNullOrEmpty(payload_name))
                {
                    if (_current_payload[chain]?.device_info?.device?.name != payload_name)
                    {
                        throw new ArgumentException($"Invalid payload name to detach from kinematic chain {chain}");
                    }
                }

                _current_payload[chain] = null;

                rrfire_payload_changed(chain, "");
                _state_seqno++;
            }

            return Task.FromResult(0);
        }

        public virtual Task<object> async_getf_param(string param_name, int rr_timeout = -1)
        {
            throw new ArgumentException("Invalid parameter");
        }

        public virtual Task async_setf_param(string param_name, object value_, int rr_timeout = -1)
        {
            throw new ArgumentException("Invalid parameter");
        }

        public virtual Task<DeviceInfo> async_get_device_info(int rr_timeout = -1)
        {
            return Task.FromResult(_robot_info.device_info);
        }

        public virtual Task<IsochInfo> async_get_isoch_info(int rr_timeout = -1)
        {

            var isoch_info = new IsochInfo();
            isoch_info.update_rate = 1.0 / _update_period;
            isoch_info.max_downsample = 1000;
            
            lock (this)
            {
                isoch_info.isoch_epoch = _stopwatch_epoch;
            }
            return Task.FromResult(isoch_info);            
        }

        public virtual Task<uint> async_get_isoch_downsample(int rr_timeout = -1)
        {
            lock(this)
            {
                return Task.FromResult(_broadcast_downsampler.GetClientDownsample(ServerEndpoint.CurrentEndpoint));
            }
        }

        public virtual Task async_set_isoch_downsample(uint value, int rr_timeout = -1)
        {
            lock(this)
            {
                _broadcast_downsampler.SetClientDownsample(ServerEndpoint.CurrentEndpoint, value);
                return Task.FromResult(0);
            }
        }

        public virtual void RRServiceObjectInit(ServerContext context, string service_path)
        {
            rrvar_robot_state_sensor_data.MaxBacklog = 3;
            _broadcast_downsampler = new BroadcastDownsampler(context, 0);
            _broadcast_downsampler.AddPipeBroadcaster(rrvar_robot_state_sensor_data);
            _broadcast_downsampler.AddWireBroadcaster(rrvar_robot_state);
            _broadcast_downsampler.AddWireBroadcaster(rrvar_advanced_robot_state);
            _broadcast_downsampler.AddWireBroadcaster(rrvar_device_clock_now);
        }
    }
    
    enum TrajectoryTaskRes
    {
        unknown = 0,
        ready,
        first_valid_setpoint,
        valid_setpoint,
        trajectory_complete,
        invalid_state,
        joint_tol_error,
        failed
    }

    public class TrajectoryTask : Generator2<com.robotraconteur.robotics.trajectory.TrajectoryStatus>
    {

        AbstractRobot parent;
        JointTrajectoryInterpolator path;
        bool next_called = false;
        bool started = false;
        long start_time = 0;
        bool aborted = false;
        bool cancelled = false;
        bool joint_tol_error = false;
        bool finished = false;
        TaskCompletionSource<int> next_wait = new TaskCompletionSource<int>();
        TaskCompletionSource<int> queue_wait = new TaskCompletionSource<int>();
        bool queued;

        uint owner_ep;

        RobotRaconteurNode node;

        public TrajectoryTask(AbstractRobot parent, JointTrajectoryInterpolator path, bool queued, uint owner_ep)
        {
            this.parent = parent;
            this.path = path;
            this.queued = queued;

            this.owner_ep = owner_ep;
            //TODO: find out which node is being used
            this.node = RobotRaconteurNode.s;

            connection_test().ContinueWith(t => { });
        }

        public void Abort()
        {
            throw new NotImplementedException();
        }

        public Task AsyncAbort(int timeout = -1)
        {
            aborted = true;
            parent._abort_trajectory(this);
            next_wait.TrySetException(new OperationAbortedException("Trajectory execution aborted"));
            return Task.FromResult(0);
            
        }

        public void Close()
        {
            throw new NotImplementedException();
        }

        public Task AsyncClose(int timeout = -1)
        {
            cancelled = true;
            parent._cancel_trajectory(this);
            next_wait.TrySetException(new OperationAbortedException("Trajectory execution cancelled"));
            return Task.FromResult(0);
        }

        bool success_sent = false;
        public async Task<TrajectoryStatus> AsyncNext(int timeout = -1)
        {
            if (success_sent)
            {
                throw new StopIterationException("");
            }

            lock(this)
            {
                bool first_call = false;
                if (!next_called)
                {
                    first_call = true;
                }
                next_called = true;

                if (first_call && queued)
                {
                    //Report back that we are queued immediately
                    var ret = new TrajectoryStatus();
                    ret.action_status = ActionStatusCode.queued;
                    ret.trajectory_time = 0;
                    ret.current_waypoint = 0;
                    ret.seqno = parent._state_seqno;
                    return ret;
                }


            }

            Task<Task> wait_task;
            if (queued)
            {
                wait_task = Task.WhenAny(Task.Delay(5000), next_wait.Task, queue_wait.Task);
            }
            else
            {
                wait_task = Task.WhenAny(Task.Delay(5000), next_wait.Task);
            }

            var wait_task1 = await wait_task;
            await wait_task1;

            if (!this.started)
            {
                // Still queued...
                var ret = new TrajectoryStatus();
                ret.action_status = ActionStatusCode.queued;
                ret.trajectory_time = 0;
                ret.current_waypoint = 0;
                ret.seqno = parent._state_seqno;
                return ret;
            }

            if (finished)
            {
                success_sent = true;
                var ret = new TrajectoryStatus();
                ret.action_status = ActionStatusCode.complete;
                ret.trajectory_time = traj_t;
                ret.current_waypoint = (uint)traj_waypoint;
                ret.seqno = parent._state_seqno;
                return ret;
            }
            else
            {
                var ret = new TrajectoryStatus();
                ret.action_status = ActionStatusCode.running;
                ret.trajectory_time = traj_t;
                ret.current_waypoint = (uint)traj_waypoint;
                ret.seqno = parent._state_seqno;
                return ret;
            }
        }

        public Task<TrajectoryStatus[]> NextAll(CancellationToken cancel = default)
        {
            throw new NotImplementedException();
        }

        internal void _cancelled_in_queue()
        {
            cancelled = true;
            next_wait.TrySetException(new OperationAbortedException("Trajectory cancelled by controller before start"));
        }

        internal void invalid_mode()
        {
            aborted = true;            
            next_wait.TrySetException(new OperationAbortedException("Invalid mode for trajectory execution"));     
        }

        double traj_t = 0;
        int traj_waypoint = 0;
        internal TrajectoryTaskRes get_setpoint(long now, double[] current_joint_pos, out double[] joint_pos, out double[] joint_vel, out double trajectory_time, out double trajectory_max_time, out int current_waypoint)
        {
            if (cancelled || aborted)
            {
                joint_pos = null;
                joint_vel = null;
                trajectory_time = 0;
                current_waypoint = 0;
                trajectory_max_time = 0;
                return TrajectoryTaskRes.failed;
            }

            bool first_call = false;

            double t = 0;

            if (next_called)
            {
                if (!started)
                {
                    start_time = now;
                    started = true;
                    first_call = true;
                }

                long t_long = now - start_time;
                t = ((double)t_long) / 1000.0;
            }

            this.path.Intepolate(t, out var joint_pos1, out var current_waypoint1);

            for (int i=0; i< current_joint_pos.Length; i++)
            {
                if (Math.Abs(current_joint_pos[i] - joint_pos1[i]) > parent._trajectory_error_tol)
                {
                    joint_tol_error = true;
                    joint_pos = null;
                    joint_vel = null;
                    trajectory_time = 0;
                    current_waypoint = 0;
                    trajectory_max_time = 0;
                    next_wait.TrySetException(new OperationFailedException("Trajectory tolerance failure"));
                    return TrajectoryTaskRes.joint_tol_error;
                }
            }

            if (!next_called)
            {
                joint_pos = null;
                joint_vel = null;
                trajectory_time = 0;
                current_waypoint = 0;
                trajectory_max_time = path.MaxTime;
                return TrajectoryTaskRes.ready;
            }

            joint_pos = joint_pos1;
            joint_vel = null;
            trajectory_time = t;
            traj_t = t;
            traj_waypoint = current_waypoint1;
            current_waypoint = current_waypoint1;
            trajectory_max_time = path.MaxTime;

            if (t > path.MaxTime)
            {
                trajectory_time = path.MaxTime;
                traj_t = path.MaxTime;
                finished = true;                
                next_wait.TrySetResult(0);
                return TrajectoryTaskRes.trajectory_complete;
            }

            if (first_call)
            {
                if (queued)
                {
                    queued = false;
                    queue_wait.TrySetResult(0);
                }
                return TrajectoryTaskRes.first_valid_setpoint;
            }
            else
            {
                return TrajectoryTaskRes.valid_setpoint;
            }
        }

        protected async Task connection_test()
        {
            while (!finished)
            {
                try
                {
                    node.CheckConnection(owner_ep);
                }
                catch (Exception)
                {
                    parent._cancel_trajectory(this);
                    next_wait.TrySetException(new ConnectionException("Connection lost"));
                }
                await Task.Delay(50);
            }
        }

        public TrajectoryStatus Next()
        {
            throw new NotImplementedException();
        }

        public TrajectoryStatus[] NextAll()
        {
            throw new NotImplementedException();
        }

        public bool TryNext(out TrajectoryStatus value)
        {
            throw new NotImplementedException();
        }
    }
}
