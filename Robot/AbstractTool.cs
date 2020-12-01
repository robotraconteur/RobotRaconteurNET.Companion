using com.robotraconteur.robotics.tool;
using System;
using System.Collections.Generic;
using System.Text;
using RobotRaconteur;
using System.Diagnostics;
using System.Threading;
using com.robotraconteur.device.isoch;
using com.robotraconteur.datetime;
using RobotRaconteur.Companion.Util;

namespace RobotRaconteur.Companion.Robot
{
    public abstract class AbstractTool : Tool_default_impl, IDisposable, IRRServiceObject
    {
        protected ToolInfo _tool_info;

        protected uint _tool_state_flags;
        
        protected double _position;
        protected double _command;
        protected double[] _sensor;
        protected Stopwatch _stopwatch;
        protected TimeSpec2 _stopwatch_epoch;
        protected BroadcastDownsampler _broadcast_downsampler;
        private bool _keep_going;
        private Thread _loop_thread;
        protected int _update_period = 10;
        protected ulong _state_seqno;
        protected com.robotraconteur.uuid.UUID _tool_uuid;

        public AbstractTool(ToolInfo tool_info)
        {
            _tool_info = tool_info;
            _tool_uuid = tool_info.device_info.device.uuid;
        }
        public virtual void Dispose()
        {
            _keep_going = false;
        }

        public virtual void RRServiceObjectInit(ServerContext context, string service_path)
        {
            _broadcast_downsampler = new BroadcastDownsampler(context, 0);
            _broadcast_downsampler.AddWireBroadcaster(rrvar_tool_state);
            _broadcast_downsampler.AddPipeBroadcaster(rrvar_tool_state_sensor_data);
            _broadcast_downsampler.AddWireBroadcaster(rrvar_device_clock_now);
        }

        public virtual void _start_tool()
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

        public virtual void _stop_tool()
        {
            _keep_going = false;
            _loop_thread.Join();
        }

        protected virtual void _loop_thread_func(object obj)
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

        public virtual void _run_timestep(long now)
        {

            ToolState rr_tool_state;
            ToolStateSensorData rr_state_sensor_data;
            BroadcastDownsamplerStep downsampler_step;
            lock (this)
            {
                downsampler_step = null;
                if (_broadcast_downsampler != null)
                {
                    downsampler_step = new BroadcastDownsamplerStep(_broadcast_downsampler);
                }
                _state_seqno++;

                _fill_state(now, out rr_tool_state);
                _fill_sensor_data(now, rr_tool_state, out rr_state_sensor_data);
            }

            _send_tool_command();

            using (downsampler_step)
            {
                _send_states(now, rr_tool_state, rr_state_sensor_data);
            }
            
        }

        protected abstract void _fill_state(long now, out ToolState rr_tool_state);

        protected virtual void _send_tool_command()
        {

        }

        protected virtual void _fill_sensor_data(long now, ToolState rr_tool_state, out ToolStateSensorData rr_state_sensor_data)
        {
            if (rr_tool_state == null)
            {
                rr_state_sensor_data = null;
                return;
            }



            var sensor_data_header = SensorDataUtil.FillSensorDataHeader(RobotRaconteurNode.s,_tool_info?.device_info, _state_seqno);
            
            var sensor_data = new ToolStateSensorData();
            sensor_data.data_header = sensor_data_header;
            sensor_data.robot_state = rr_tool_state;

            rr_state_sensor_data = sensor_data;
        }

        protected virtual void _send_states(long now, ToolState rr_tool_state, ToolStateSensorData rr_state_sensor_data)
        {
            if (rrvar_tool_state != null)
            {
                rrvar_tool_state.OutValue = rr_tool_state;
            }

            rrvar_tool_state_sensor_data?.AsyncSendPacket(rr_state_sensor_data).ContinueWith(t => { var ignore = t.Exception; }, System.Threading.Tasks.TaskContinuationOptions.OnlyOnFaulted);

            if (rrvar_device_clock_now != null)
            {
                rrvar_device_clock_now.OutValue = DateTimeUtil.FillDeviceTime(RobotRaconteurNode.s, tool_info?.device_info, _state_seqno);
            }
        }

        public virtual IsochInfo get_isoch_info
        {
            get
            {
                var isoch_info = new IsochInfo();
                isoch_info.update_rate = 1.0 / _update_period;
                isoch_info.max_downsample = 1000;
                
                lock (this)
                {
                    isoch_info.isoch_epoch = _stopwatch_epoch;
                }
                
                return isoch_info;
            }
        }

        public override uint isoch_downsample
        {
            get
            {
                lock (this)
                {
                    return _broadcast_downsampler.GetClientDownsample(ServerEndpoint.CurrentEndpoint);
                }
            }
            set
            {
                lock(this)
                {
                    _broadcast_downsampler.SetClientDownsample(ServerEndpoint.CurrentEndpoint, value);
                }
            }
        }

        public override void halt()
        {
            
        }
    }
}
