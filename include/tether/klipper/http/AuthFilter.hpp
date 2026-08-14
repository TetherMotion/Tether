#pragma once

/// @file AuthFilter.hpp
/// @brief Drogon HttpFilter that enforces authentication on all routes.

#include "tether/klipper/http/KlippyHttpServer.hpp"

#include <drogon/HttpFilter.h>

namespace tether::klipper::http {

/// @brief Drogon filter that checks authentication before allowing requests.
///
/// Applied to all REST routes. If auth is required and the request fails
/// the auth check, a 401 Unauthorized response is returned immediately.
class AuthFilter : public drogon::HttpFilter<AuthFilter, false> {
public:
    explicit AuthFilter(KlippyHttpServer* server) : server_(server) {}

    virtual void doFilter(const drogon::HttpRequestPtr& req,
                          drogon::FilterCallback&& fcb,
                          drogon::FilterChainCallback&& fccb) override {
        if (!server_) {
            fccb();
            return;
        }

        // Handle CORS preflight (OPTIONS) without auth
        if (req->method() == drogon::HttpMethod::Options) {
            fccb();
            return;
        }

        if (server_->checkAuth(req)) {
            fccb();
        } else {
            auto resp = drogon::HttpResponse::newHttpResponse();
            resp->setStatusCode(drogon::HttpStatusCode::k401Unauthorized);
            resp->setContentTypeCode(drogon::CT_APPLICATION_JSON);
            resp->setBody(R"({"error":{"code":401,"message":"Unauthorized"}})");
            server_->addCorsHeaders(resp, req);
            fcb(resp);
        }
    }

private:
    KlippyHttpServer* server_;
};

} // namespace tether::klipper::http
