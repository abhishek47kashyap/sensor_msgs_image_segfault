# Description
This repository is a minimum reproducible example of a segfault from accessing the `frame_id` of `sensor_msgs/Image`'s `header` field as received from a service server. This example was created and tested on ROS2 Humble.

This minimum reproducible example complements the question asked here: [Service response sensor_msgs/Image cannot be retrieved](https://robotics.stackexchange.com/questions/115197/service-response-sensor-msgs-image-cannot-be-retrieved)

The [client](https://github.com/abhishek47kashyap/sensor_msgs_image_segfault/blob/main/src/client.cpp) makes a service request to obtain the newest RGBD that the [server](https://github.com/abhishek47kashyap/sensor_msgs_image_segfault/blob/main/src/server.cpp) has on record. The service definition [`GetRGBD.srv`](https://github.com/abhishek47kashyap/sensor_msgs_image_segfault/blob/main/srv/GetRGBD.srv) has `sensor_msgs/Image` for RGB and depth, and both of them have their `header.frame_id` corrupted(?) on the client side but not on the server side.

# Reproducing the segfault
## Prerequisites
- It's assumed ROS2 Humble is available and that `source /opt/ros/humble/setup.bash` is in `~/.bashrc`.
- We will grab a RealSense D435i bag file ([sample recordings](https://github.com/IntelRealSense/librealsense/blob/master/doc/sample-data.md#files)) "_Outdoor scene with D435i pre-production sample (Depth from Stereo with IMU)_", it's the one at the very bottom of the table. Click on the image to its left or just click [here](https://librealsense.intel.com/rs-tests/TestData/d435i_sample_data.zip) to begin the download. It's a 451 MB zip file. Unzip it and there will be two bag files, we only need one so let's go with `d435i_walking.bag`.
## Commands to execute
### Cloning this repository and building the workspace
```
mkdir -p segfault_ws/src && cd segfault_ws/src
git clone https://github.com/abhishek47kashyap/sensor_msgs_image_segfault.git segfault_pkg
cd ..
rosdep update && rosdep install --from-paths src -y --ignore-src
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Launching nodes and making the service request to observe segfault
We will use a total of three terminals:
1. In terminal 1, source the workspace we built in the previous step and run `ros2 launch segfault_pkg server_client_launch.py`. Expect to see a log in green saying `Ready to accept trigger request!`.
2. Grab full filepath of the bagfile `d435i_walking.bag` and in terminal 2 run `ros2 launch realsense2_camera rs_launch.py rosbag_filename:='/full/path/to/d435i_walking.bag'`. In terminal 1 you should see these logs:
   ```
   First depth image rcvd!
   Acquired camera parameters!
   First color image rcvd!
   Acquired first RGBD frame
   First RGBD header frame_id: camera_color_optical_frame
   ```
   These logs confirm that [`server.cpp`](https://github.com/abhishek47kashyap/sensor_msgs_image_segfault/blob/main/src/server.cpp) had its subscription callbacks triggered by incoming messages from the bag file.
3. Finally, in terminal 3, ensure output of `ros2 service find std_srvs/srv/Trigger` produces `/example_segfault/trigger`. Then make the service request `ros2 service call /example_segfault/trigger std_srvs/srv/Trigger {}`. You will very likely observe a segfault in terminal 1 with the stacktrace printed out.

<details>
  <summary>Example stacktrace</summary>

  ```js
[segfault_pkg_node-1] Stack trace (most recent call last) in thread 300863:
[segfault_pkg_node-1] #14   Object "", at 0xffffffffffffffff, in 
[segfault_pkg_node-1] #13   Source "../sysdeps/unix/sysv/linux/x86_64/clone3.S", line 81, in __clone3 [0x79f8d9d2684f]
[segfault_pkg_node-1] #12   Source "./nptl/pthread_create.c", line 442, in start_thread [0x79f8d9c94ac2]
[segfault_pkg_node-1] #11   Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x79f8da2dc252, in 
[segfault_pkg_node-1] #10   Object "/opt/ros/humble/lib/librclcpp.so", at 0x79f8da0f83b9, in rclcpp::executors::MultiThreadedExecutor::run(unsigned long)
[segfault_pkg_node-1] #9    Object "/opt/ros/humble/lib/librclcpp.so", at 0x79f8da0f1065, in rclcpp::Executor::execute_any_executable(rclcpp::AnyExecutable&)
[segfault_pkg_node-1] #8    Object "/opt/ros/humble/lib/librclcpp.so", at 0x79f8da0f0cf9, in rclcpp::Executor::execute_service(std::shared_ptr<rclcpp::ServiceBase>)
[segfault_pkg_node-1] #7    Object "/opt/ros/humble/lib/librclcpp.so", at 0x79f8da0f3315, in 
[segfault_pkg_node-1] #6    Source "/opt/ros/humble/include/rclcpp/rclcpp/service.hpp", line 473, in handle_request [0x6426901de65d]
[segfault_pkg_node-1]         470:     std::shared_ptr<void> request) override
[segfault_pkg_node-1]         471:   {
[segfault_pkg_node-1]         472:     auto typed_request = std::static_pointer_cast<typename ServiceT::Request>(request);
[segfault_pkg_node-1]       > 473:     auto response = any_callback_.dispatch(this->shared_from_this(), request_header, typed_request);
[segfault_pkg_node-1]         474:     if (response) {
[segfault_pkg_node-1]         475:       send_response(*request_header, *response);
[segfault_pkg_node-1]         476:     }
[segfault_pkg_node-1] #5  | Source "/opt/ros/humble/include/rclcpp/rclcpp/any_service_callback.hpp", line 180, in operator()
[segfault_pkg_node-1]     |   178:       (void)request_header;
[segfault_pkg_node-1]     |   179:       const auto & cb = std::get<SharedPtrCallback>(callback_);
[segfault_pkg_node-1]     | > 180:       cb(std::move(request), response);
[segfault_pkg_node-1]     |   181:     } else if (std::holds_alternative<SharedPtrWithRequestHeaderCallback>(callback_)) {
[segfault_pkg_node-1]     |   182:       const auto & cb = std::get<SharedPtrWithRequestHeaderCallback>(callback_);
[segfault_pkg_node-1]       Source "/usr/include/c++/10/bits/std_function.h", line 622, in dispatch [0x6426901de0b6]
[segfault_pkg_node-1]         619:     {
[segfault_pkg_node-1]         620:       if (_M_empty())
[segfault_pkg_node-1]         621:      __throw_bad_function_call();
[segfault_pkg_node-1]       > 622:       return _M_invoker(_M_functor, std::forward<_ArgTypes>(__args)...);
[segfault_pkg_node-1]         623:     }
[segfault_pkg_node-1]         624: 
[segfault_pkg_node-1]         625: #if __cpp_rtti
[segfault_pkg_node-1] #4  | Source "/usr/include/c++/10/bits/std_function.h", line 291, in __invoke_r<void, std::_Bind<void (GetRGBDClient::*(GetRGBDClient*, std::_Placeholder<1>, std::_Placeholder<2>))(std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > >)>&, std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > > >
[segfault_pkg_node-1]     |   289:       _M_invoke(const _Any_data& __functor, _ArgTypes&&... __args)
[segfault_pkg_node-1]     |   290:       {
[segfault_pkg_node-1]     | > 291:      return std::__invoke_r<_Res>(*_Base::_M_get_pointer(__functor),
[segfault_pkg_node-1]     |   292:                                   std::forward<_ArgTypes>(__args)...);
[segfault_pkg_node-1]     |   293:       }
[segfault_pkg_node-1]     | Source "/usr/include/c++/10/bits/invoke.h", line 110, in __invoke_impl<void, std::_Bind<void (GetRGBDClient::*(GetRGBDClient*, std::_Placeholder<1>, std::_Placeholder<2>))(std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > >)>&, std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > > >
[segfault_pkg_node-1]     |   108:       using __tag = typename __result::__invoke_type;
[segfault_pkg_node-1]     |   109:       if constexpr (is_void_v<_Res>)
[segfault_pkg_node-1]     | > 110:      std::__invoke_impl<__type>(__tag{}, std::forward<_Callable>(__fn),
[segfault_pkg_node-1]     |   111:                                    std::forward<_Args>(__args)...);
[segfault_pkg_node-1]     |   112:       else
[segfault_pkg_node-1]     | Source "/usr/include/c++/10/bits/invoke.h", line 60, in operator()<std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > > >
[segfault_pkg_node-1]     |    58:     constexpr _Res
[segfault_pkg_node-1]     |    59:     __invoke_impl(__invoke_other, _Fn&& __f, _Args&&... __args)
[segfault_pkg_node-1]     | >  60:     { return std::forward<_Fn>(__f)(std::forward<_Args>(__args)...); }
[segfault_pkg_node-1]     |    61: 
[segfault_pkg_node-1]     |    62:   template<typename _Res, typename _MemFun, typename _Tp, typename... _Args>
[segfault_pkg_node-1]     | Source "/usr/include/c++/10/functional", line 499, in __call<void, std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >&&, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > >&&, 0, 1, 2>
[segfault_pkg_node-1]     |   497:      operator()(_Args&&... __args)
[segfault_pkg_node-1]     |   498:      {
[segfault_pkg_node-1]     | > 499:        return this->__call<_Result>(
[segfault_pkg_node-1]     |   500:            std::forward_as_tuple(std::forward<_Args>(__args)...),
[segfault_pkg_node-1]     |   501:            _Bound_indexes());
[segfault_pkg_node-1]     | Source "/usr/include/c++/10/functional", line 416, in __invoke<void (GetRGBDClient::*&)(std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > >), GetRGBDClient*&, std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > > >
[segfault_pkg_node-1]     |   414:      __call(tuple<_Args...>&& __args, _Index_tuple<_Indexes...>)
[segfault_pkg_node-1]     |   415:      {
[segfault_pkg_node-1]     | > 416:        return std::__invoke(_M_f,
[segfault_pkg_node-1]     |   417:            _Mu<_Bound_args>()(std::get<_Indexes>(_M_bound_args), __args)...
[segfault_pkg_node-1]     |   418:            );
[segfault_pkg_node-1]     | Source "/usr/include/c++/10/bits/invoke.h", line 95, in __invoke_impl<void, void (GetRGBDClient::*&)(std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > >), GetRGBDClient*&, std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > > >
[segfault_pkg_node-1]     |    93:       using __type = typename __result::type;
[segfault_pkg_node-1]     |    94:       using __tag = typename __result::__invoke_type;
[segfault_pkg_node-1]     | >  95:       return std::__invoke_impl<__type>(__tag{}, std::forward<_Callable>(__fn),
[segfault_pkg_node-1]     |    96:                                    std::forward<_Args>(__args)...);
[segfault_pkg_node-1]     |    97:     }
[segfault_pkg_node-1]       Source "/usr/include/c++/10/bits/invoke.h", line 73, in _M_invoke [0x6426901c0f8b]
[segfault_pkg_node-1]          70:     __invoke_impl(__invoke_memfun_deref, _MemFun&& __f, _Tp&& __t,
[segfault_pkg_node-1]          71:                _Args&&... __args)
[segfault_pkg_node-1]          72:     {
[segfault_pkg_node-1]       >  73:       return ((*std::forward<_Tp>(__t)).*__f)(std::forward<_Args>(__args)...);
[segfault_pkg_node-1]          74:     }
[segfault_pkg_node-1]          75: 
[segfault_pkg_node-1]          76:   template<typename _Res, typename _MemPtr, typename _Tp>
[segfault_pkg_node-1] #3  | Source "/home/abhishek/Code/random/frame_id_segfault_ws/src/segfault_pkg/src/client.cpp", line 75, in operator<< <char, std::char_traits<char>, std::allocator<char> >
[segfault_pkg_node-1]     |    73:             RCLCPP_INFO_STREAM(logger_, "CLIENT: is_bigendian: " << response->rgb.is_bigendian);
[segfault_pkg_node-1]     |    74:             RCLCPP_INFO(logger_, "CLIENT: About to log header frame ID..");
[segfault_pkg_node-1]     | >  75:             RCLCPP_INFO_STREAM(logger_, "CLIENT: header frame ID: " << response->rgb.header.frame_id);  // <- segfaults
[segfault_pkg_node-1]     |    76:             const sensor_msgs::msg::Image rgb_img = response->rgb;
[segfault_pkg_node-1]     |    77:             RCLCPP_INFO(logger_, "CLIENT: SUCCESS");
[segfault_pkg_node-1]       Source "/usr/include/c++/10/bits/basic_string.h", line 6485, in callbackTrigger [0x6426901cd166]
[segfault_pkg_node-1]        6482:     {
[segfault_pkg_node-1]        6483:       // _GLIBCXX_RESOLVE_LIB_DEFECTS
[segfault_pkg_node-1]        6484:       // 586. string inserter not a formatted function
[segfault_pkg_node-1]       >6485:       return __ostream_insert(__os, __str.data(), __str.size());
[segfault_pkg_node-1]        6486:     }
[segfault_pkg_node-1]        6487: 
[segfault_pkg_node-1]        6488:   /**
[segfault_pkg_node-1] #2    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x79f8da33cb64, in std::basic_ostream<char, std::char_traits<char> >& std::__ostream_insert<char, std::char_traits<char> >(std::basic_ostream<char, std::char_traits<char> >&, char const*, long)
[segfault_pkg_node-1] #1    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x79f8da34a8ad, in std::basic_streambuf<char, std::char_traits<char> >::xsputn(char const*, long)
[segfault_pkg_node-1] #0    Source "../sysdeps/x86_64/multiarch/memmove-vec-unaligned-erms.S", line 394, in __memmove_avx512_unaligned_erms [0x79f8d9da6840]
[segfault_pkg_node-1] Segmentation fault (Signal sent by the kernel [(nil)])
[ERROR] [segfault_pkg_node-1]: process has died [pid 300838, exit code -11, cmd '/home/abhishek/Code/random/frame_id_segfault_ws/install/segfault_pkg/lib/segfault_pkg/segfault_pkg_node --ros-args -r __ns:=/example_segfault'].
  ```
</details>

If however there is no segfault and you observe `CLIENT: SUCCESS` you're living a better life than me and I want to appreciate the time you took reading this far 🙂

## Comprehending the logs
In terminal 1, relevant logs generated on the server side have the prefix `SERVER` and those generated on the client side have `CLIENT`.

On the _server_ side, make note of:
```
Service get_rgbd was called!
SERVER: Assigned RGB and Depth, height = 480, width = 640, encoding: rgb8, row length = 1920
SERVER: Data size: 921600
SERVER: Frame ID: camera_color_optical_frame
SERVER: size of color image height = 480, width = 640, encoding: rgb8, row length = 1920
SERVER: is_bigendian:
Completed get_rgbd service request, frame ID: camera_color_optical_frame
```

On the _client_ side, observe:
```
CLIENT: RGBD and camera pose acquisition status: true
CLIENT: Size of color image height = 480, width = 640, encoding: rgb8, row length = 1920
CLIENT: Data size: 921600
CLIENT: is_bigendian:
CLIENT: About to log header frame ID..
```

It's interesting that `frame_id` can be logged on the server side yet attempting to do so on client side causes a segfault. It's because of this segfault that `rgb` and `depth` cannot be copied over to another variable or forwarded on as part of another service request.
