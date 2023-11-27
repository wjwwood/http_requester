# http_requester

This package provides a ROS Node with a ROS Service for making HTTP requests.

The implementation uses libcurl to make the requests, and supports GET and POST.

## Nodes

This package provides the `http_requester::HTTPRequesterNode` ROS Node as a component, for example:

```
ros2 component load /ComponentManager http_requester http_requester::HTTPRequesterNode
```

Assuming you have a component manager called `/ComponentManager` running, see:

- [https://docs.ros.org/en/rolling/Tutorials/Intermediate/Composition.html#run-time-composition-using-ros-services-with-a-publisher-and-subscriber](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Composition.html#run-time-composition-using-ros-services-with-a-publisher-and-subscriber)

## Executables

This package provides the `http_requester` executable for running the ROS Node stand-alone:

```
ros2 run http_requester http_requester
```

## ROS Services

The ROS Node provides the ROS Service `~/make_http_request`.

### Example GET HTTP Request

```
ros2 service call \
  /http_requester/make_http_request \
  http_requester_interfaces/srv/MakeHTTPRequest \
  '{method: "GET", url: "www.example.com"}'
```

### Example POST HTTP Request

```
ros2 service call \
  /http_requester/make_http_request \
  http_requester_interfaces/srv/MakeHTTPRequest \
  '{method: "POST", url: "https://httpbin.org/post", payload: "this is a test"}'
```
