# Description
This repository is a minimum reproducible example of a segfault from accessing `std_msgs/String` in the service response as received from a service server. This example was created and tested on ROS2 Humble.

The [client](https://github.com/abhishek47kashyap/sensor_msgs_image_segfault/blob/string/src/client.cpp) makes a service request to obtain a string from the [server](https://github.com/abhishek47kashyap/sensor_msgs_image_segfault/blob/string/src/server.cpp). The service definition [`GetString.srv`](https://github.com/abhishek47kashyap/sensor_msgs_image_segfault/blob/string/srv/GetString.srv) has `input` and `output` string fields, and `output` is corrupted(?) on the client side but not on the server side.

# Reproducing the segfault
## Clone this repository and build the workspace
```
mkdir -p segfault_ws/src && cd segfault_ws/src
git clone https://github.com/abhishek47kashyap/sensor_msgs_image_segfault.git segfault_pkg
cd ..
rosdep update && rosdep install --from-paths src -y --ignore-src
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## Launching nodes and making the service request to observe segfault
We will use a total of two terminals:
1. In terminal 1, source the workspace we built in the previous step and run `ros2 launch segfault_pkg server_client_launch.py`. Expect to see a log in green saying `Ready to accept trigger request!`.
2. In terminal 2, ensure output of `ros2 service find std_srvs/srv/Trigger` produces `/example_segfault/trigger`. Then make the service request `ros2 service call /example_segfault/trigger std_srvs/srv/Trigger {}`. You will very likely observe a segfault in terminal 1 with the stacktrace printed out.

<details>
  <summary>Example stacktrace</summary>

  ```js
[segfault_pkg_node-1] Stack trace (most recent call last) in thread 82553:
[segfault_pkg_node-1] #21   Object "", at 0xffffffffffffffff, in 
[segfault_pkg_node-1] #20   Source "../sysdeps/unix/sysv/linux/x86_64/clone3.S", line 81, in __clone3 [0x76dc24b2684f]
[segfault_pkg_node-1] #19   Source "./nptl/pthread_create.c", line 442, in start_thread [0x76dc24a94ac2]
[segfault_pkg_node-1] #18   Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x76dc250dc252, in 
[segfault_pkg_node-1] #17   Object "/opt/ros/humble/lib/librclcpp.so", at 0x76dc24ef83b9, in rclcpp::executors::MultiThreadedExecutor::run(unsigned long)
[segfault_pkg_node-1] #16   Object "/opt/ros/humble/lib/librclcpp.so", at 0x76dc24ef1065, in rclcpp::Executor::execute_any_executable(rclcpp::AnyExecutable&)
[segfault_pkg_node-1] #15   Object "/opt/ros/humble/lib/librclcpp.so", at 0x76dc24ef0cf9, in rclcpp::Executor::execute_service(std::shared_ptr<rclcpp::ServiceBase>)
[segfault_pkg_node-1] #14   Object "/opt/ros/humble/lib/librclcpp.so", at 0x76dc24ef3315, in 
[segfault_pkg_node-1] #13   Source "/opt/ros/humble/include/rclcpp/rclcpp/service.hpp", line 473, in handle_request [0x5fc5c45f26e0]
[segfault_pkg_node-1]         470:     std::shared_ptr<void> request) override
[segfault_pkg_node-1]         471:   {
[segfault_pkg_node-1]         472:     auto typed_request = std::static_pointer_cast<typename ServiceT::Request>(request);
[segfault_pkg_node-1]       > 473:     auto response = any_callback_.dispatch(this->shared_from_this(), request_header, typed_request);
[segfault_pkg_node-1]         474:     if (response) {
[segfault_pkg_node-1]         475:       send_response(*request_header, *response);
[segfault_pkg_node-1]         476:     }
[segfault_pkg_node-1] #12   Source "/opt/ros/humble/include/rclcpp/rclcpp/any_service_callback.hpp", line 180, in dispatch [0x5fc5c45f3b8d]
[segfault_pkg_node-1]         177:     if (std::holds_alternative<SharedPtrCallback>(callback_)) {
[segfault_pkg_node-1]         178:       (void)request_header;
[segfault_pkg_node-1]         179:       const auto & cb = std::get<SharedPtrCallback>(callback_);
[segfault_pkg_node-1]       > 180:       cb(std::move(request), response);
[segfault_pkg_node-1]         181:     } else if (std::holds_alternative<SharedPtrWithRequestHeaderCallback>(callback_)) {
[segfault_pkg_node-1]         182:       const auto & cb = std::get<SharedPtrWithRequestHeaderCallback>(callback_);
[segfault_pkg_node-1]         183:       cb(request_header, std::move(request), response);
[segfault_pkg_node-1] #11   Source "/usr/include/c++/10/bits/std_function.h", line 622, in operator() [0x5fc5c45f59fc]
[segfault_pkg_node-1]         619:     {
[segfault_pkg_node-1]         620:       if (_M_empty())
[segfault_pkg_node-1]         621:      __throw_bad_function_call();
[segfault_pkg_node-1]       > 622:       return _M_invoker(_M_functor, std::forward<_ArgTypes>(__args)...);
[segfault_pkg_node-1]         623:     }
[segfault_pkg_node-1]         624: 
[segfault_pkg_node-1]         625: #if __cpp_rtti
[segfault_pkg_node-1] #10   Source "/usr/include/c++/10/bits/std_function.h", line 291, in _M_invoke [0x5fc5c45e3620]
[segfault_pkg_node-1]         288:       static _Res
[segfault_pkg_node-1]         289:       _M_invoke(const _Any_data& __functor, _ArgTypes&&... __args)
[segfault_pkg_node-1]         290:       {
[segfault_pkg_node-1]       > 291:      return std::__invoke_r<_Res>(*_Base::_M_get_pointer(__functor),
[segfault_pkg_node-1]         292:                                   std::forward<_ArgTypes>(__args)...);
[segfault_pkg_node-1]         293:       }
[segfault_pkg_node-1]         294:     };
[segfault_pkg_node-1] #9    Source "/usr/include/c++/10/bits/invoke.h", line 110, in __invoke_r<void, std::_Bind<void (GetStringClient::*(GetStringClient*, std::_Placeholder<1>, std::_Placeholder<2>))(std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > >)>&, std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > > > [0x5fc5c45e55a6]
[segfault_pkg_node-1]         107:       using __type = typename __result::type;
[segfault_pkg_node-1]         108:       using __tag = typename __result::__invoke_type;
[segfault_pkg_node-1]         109:       if constexpr (is_void_v<_Res>)
[segfault_pkg_node-1]       > 110:      std::__invoke_impl<__type>(__tag{}, std::forward<_Callable>(__fn),
[segfault_pkg_node-1]         111:                                      std::forward<_Args>(__args)...);
[segfault_pkg_node-1]         112:       else
[segfault_pkg_node-1]         113:      return std::__invoke_impl<__type>(__tag{},
[segfault_pkg_node-1] #8    Source "/usr/include/c++/10/bits/invoke.h", line 60, in __invoke_impl<void, std::_Bind<void (GetStringClient::*(GetStringClient*, std::_Placeholder<1>, std::_Placeholder<2>))(std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > >)>&, std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > > > [0x5fc5c45e6968]
[segfault_pkg_node-1]          57:   template<typename _Res, typename _Fn, typename... _Args>
[segfault_pkg_node-1]          58:     constexpr _Res
[segfault_pkg_node-1]          59:     __invoke_impl(__invoke_other, _Fn&& __f, _Args&&... __args)
[segfault_pkg_node-1]       >  60:     { return std::forward<_Fn>(__f)(std::forward<_Args>(__args)...); }
[segfault_pkg_node-1]          61: 
[segfault_pkg_node-1]          62:   template<typename _Res, typename _MemFun, typename _Tp, typename... _Args>
[segfault_pkg_node-1]          63:     constexpr _Res
[segfault_pkg_node-1] #7    Source "/usr/include/c++/10/functional", line 499, in operator()<std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > > > [0x5fc5c45e8470]
[segfault_pkg_node-1]         496:      _Result
[segfault_pkg_node-1]         497:      operator()(_Args&&... __args)
[segfault_pkg_node-1]         498:      {
[segfault_pkg_node-1]       > 499:        return this->__call<_Result>(
[segfault_pkg_node-1]         500:            std::forward_as_tuple(std::forward<_Args>(__args)...),
[segfault_pkg_node-1]         501:            _Bound_indexes());
[segfault_pkg_node-1]         502:      }
[segfault_pkg_node-1] #6    Source "/usr/include/c++/10/functional", line 416, in __call<void, std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >&&, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > >&&, 0, 1, 2> [0x5fc5c45e9a56]
[segfault_pkg_node-1]         413:      _Result
[segfault_pkg_node-1]         414:      __call(tuple<_Args...>&& __args, _Index_tuple<_Indexes...>)
[segfault_pkg_node-1]         415:      {
[segfault_pkg_node-1]       > 416:        return std::__invoke(_M_f,
[segfault_pkg_node-1]         417:            _Mu<_Bound_args>()(std::get<_Indexes>(_M_bound_args), __args)...
[segfault_pkg_node-1]         418:            );
[segfault_pkg_node-1]         419:      }
[segfault_pkg_node-1] #5    Source "/usr/include/c++/10/bits/invoke.h", line 95, in __invoke<void (GetStringClient::*&)(std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > >), GetStringClient*&, std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > > > [0x5fc5c45eb566]
[segfault_pkg_node-1]          92:       using __result = __invoke_result<_Callable, _Args...>;
[segfault_pkg_node-1]          93:       using __type = typename __result::type;
[segfault_pkg_node-1]          94:       using __tag = typename __result::__invoke_type;
[segfault_pkg_node-1]       >  95:       return std::__invoke_impl<__type>(__tag{}, std::forward<_Callable>(__fn),
[segfault_pkg_node-1]          96:                                      std::forward<_Args>(__args)...);
[segfault_pkg_node-1]          97:     }
[segfault_pkg_node-1] #4    Source "/usr/include/c++/10/bits/invoke.h", line 73, in __invoke_impl<void, void (GetStringClient::*&)(std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > >), GetStringClient*&, std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void> > >, std::shared_ptr<std_srvs::srv::Trigger_Response_<std::allocator<void> > > > [0x5fc5c45ed82c]
[segfault_pkg_node-1]          70:     __invoke_impl(__invoke_memfun_deref, _MemFun&& __f, _Tp&& __t,
[segfault_pkg_node-1]          71:                _Args&&... __args)
[segfault_pkg_node-1]          72:     {
[segfault_pkg_node-1]       >  73:       return ((*std::forward<_Tp>(__t)).*__f)(std::forward<_Args>(__args)...);
[segfault_pkg_node-1]          74:     }
[segfault_pkg_node-1]          75: 
[segfault_pkg_node-1]          76:   template<typename _Res, typename _MemPtr, typename _Tp>
[segfault_pkg_node-1] #3    Source "/home/abhishek/Code/random/frame_id_segfault_ws/src/segfault_pkg/src/client.cpp", line 71, in callbackTrigger [0x5fc5c45d1ad0]
[segfault_pkg_node-1]          68:         if (response->success)
[segfault_pkg_node-1]          69:         {
[segfault_pkg_node-1]          70:             RCLCPP_INFO(logger_, "CLIENT: About to log output data ..");
[segfault_pkg_node-1]       >  71:             RCLCPP_INFO_STREAM(logger_, "CLIENT: txt: " << response->output.data);
[segfault_pkg_node-1]          72:             RCLCPP_INFO(logger_, "CLIENT: SUCCESS");
[segfault_pkg_node-1]          73:         }
[segfault_pkg_node-1]          74:         else
[segfault_pkg_node-1] #2    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x76dc2513cb64, in std::basic_ostream<char, std::char_traits<char> >& std::__ostream_insert<char, std::char_traits<char> >(std::basic_ostream<char, std::char_traits<char> >&, char const*, long)
[segfault_pkg_node-1] #1    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x76dc2514a847, in std::basic_streambuf<char, std::char_traits<char> >::xsputn(char const*, long)
[segfault_pkg_node-1] #0    Source "../sysdeps/x86_64/multiarch/memmove-vec-unaligned-erms.S", line 370, in __memmove_avx512_unaligned_erms [0x76dc24ba682a]
[segfault_pkg_node-1] Segmentation fault (Address not mapped to object [0x76dc99be95e0])
[ERROR] [segfault_pkg_node-1]: process has died [pid 82541, exit code -11, cmd '/home/abhishek/Code/random/frame_id_segfault_ws/install/segfault_pkg/lib/segfault_pkg/segfault_pkg_node --ros-args -r __ns:=/example_segfault'].
  ```
</details>

If however there is no segfault and you observe `CLIENT: SUCCESS` you're living a better life than me and I want to appreciate the time you took reading this far 🙂

## Comprehending the logs
In terminal 1, relevant logs generated on the server side have the prefix `SERVER` and those generated on the client side have `CLIENT`.

On the _server_ side, make note of:
```
SERVER: Service string was called! Txt: Hello there!
SERVER: Completed string service request, responding with General Kenobi!!
```

On the _client_ side, observe:
```
CLIENT: Sending request ..
CLIENT: Service request status: true
CLIENT: About to log output data ..
```

Why can `output` can be logged on the server side yet attempting to do so on client side causes a segfault?
