#pragma once
#include "esp_err.h"
#include "esp_http_server.h"


esp_err_t init_camera();
esp_err_t jpg_stream_httpd_handler(httpd_req_t *req);