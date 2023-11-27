// Copyright 2023 William Woodall
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

#include <curl/curl.h>

#include <rclcpp/rclcpp.hpp>
#include <http_requester_interfaces/srv/make_http_request.hpp>

namespace http_requester
{

extern "C" {
// libcurl write function for appending data to the result as it is received
size_t
write_func(void * buffer, size_t size, size_t nmemb, void * userp)
{
  try {
    if (nullptr == buffer) {
      throw std::runtime_error("buffer unexpectedly nullptr");
    }
    auto & response_ref = *reinterpret_cast<std::string *>(userp);

    size_t real_size = size * nmemb;
    std::string new_data(static_cast<char *>(buffer), real_size);
    response_ref += new_data;
    return real_size;
  } catch (const std::exception & exec) {
    fprintf(stderr, "error reading data from http request: %s\n", exec.what());
    return 0;
  } catch (...) {
    fprintf(stderr, "unhandled exception in reading data from http request\n");
    return 0;
  }
}
}  // extern "C"

class HTTPRequesterNode : public rclcpp::Node
{
public:
  explicit HTTPRequesterNode(const rclcpp::NodeOptions & node_options)
  : Node("http_requester", node_options)
  {
    curl_handle_ = curl_easy_init();

    using RequestT = std::shared_ptr<MakeHTTPRequest::Request>;
    using ResponseT = std::shared_ptr<MakeHTTPRequest::Response>;
    auto callback =
      [this](RequestT request, ResponseT response) {
        RCLCPP_INFO(
          this->get_logger(),
          "Making HTTP reqeust, method: '%s', url: '%s', payload: '%s'",
          request->method.c_str(),
          request->url.c_str(),
          request->payload.c_str());

        if (request->method == "POST") {
          curl_easy_setopt(curl_handle_, CURLOPT_POSTFIELDS, request->payload.c_str());
          curl_easy_setopt(curl_handle_, CURLOPT_POSTFIELDSIZE, request->payload.size());
        } else if (request->method != "GET") {
          response->status_code = 0;
          response->response =
            "Unsupported method '" + request->method + "', only POST or GET is supported.";
          RCLCPP_ERROR(this->get_logger(), "%s", response->response.c_str());
          return;
        }

        curl_easy_setopt(curl_handle_, CURLOPT_URL, request->url.c_str());
        curl_easy_setopt(curl_handle_, CURLOPT_WRITEFUNCTION, write_func);

        char error_buffer[CURL_ERROR_SIZE] = "";
        curl_easy_setopt(curl_handle_, CURLOPT_ERRORBUFFER, error_buffer);

        curl_easy_setopt(curl_handle_, CURLOPT_WRITEDATA, &response->response);

        CURLcode ret = curl_easy_perform(curl_handle_);
        curl_easy_getinfo(curl_handle_, CURLINFO_RESPONSE_CODE, &response->status_code);
        if (response->status_code != 200 || ret != CURLE_OK) {
          RCLCPP_WARN(
            this->get_logger(),
            "HTTP request failed (%d): %s", response->status_code, error_buffer);
          response->response = std::string(error_buffer, strnlen(error_buffer, CURL_ERROR_SIZE));
        }
      };
    service_ = this->create_service<MakeHTTPRequest>("~/make_http_request", callback);

    RCLCPP_INFO(this->get_logger(), "Ready to serve http requests...");
  }

  virtual ~HTTPRequesterNode()
  {
    if (curl_handle_) {
      curl_easy_cleanup(curl_handle_);
    }
  }

private:
  using MakeHTTPRequest = http_requester_interfaces::srv::MakeHTTPRequest;
  using ServiceT = rclcpp::Service<MakeHTTPRequest>;
  std::shared_ptr<ServiceT> service_;
  CURL * curl_handle_;
};

class CURLStaticInit
{
public:
  CURLStaticInit()
  : initialized_(false)
  {
    CURLcode ret = curl_global_init(CURL_GLOBAL_ALL);
    if (ret != 0) {
      fprintf(stderr, "Error initializing libcurl! retcode = %d", ret);
    } else {
      initialized_ = true;
    }
  }

  ~CURLStaticInit()
  {
    if (initialized_) {
      curl_global_cleanup();
    }
  }

  bool initialized_;
};
static CURLStaticInit g_curl_init;

}  // namespace http_requester

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(http_requester::HTTPRequesterNode)
