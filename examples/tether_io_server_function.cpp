/**
 * @file tether_io_server_function.cpp
 * @brief Server-hosted functions: Registry::addFunction + callFunction.
 *
 * The forward RPC direction — the client calls a function the server
 * registered in its catalog. This example hosts `math.divide` with:
 *   - two required F64 args (a, b)
 *   - one optional U8 arg (precision, default 2) — optionals must trail
 *     required params and carry defaults (FunctionEntry::validSignature)
 *   - a String return — variable-length values ride the same slot
 *   - an error path: FunctionCallResult{success=false, errorMessage}
 *
 *   ./tether_io_server_function [--port 42782]
 */

#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/SchemaDigest.hpp"
#include "tether/io/SchemaValueBuilder.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/TetherIOClient.hpp"

#include <cstring>
#include <cstdio>
#include <string>

using namespace tether::io;

namespace {

SchemaKey demoKey(uint8_t tag) { SchemaKey k{}; k[0] = 0xDE; k[1] = tag; return k; }

TetherIOClient::FunctionArg argF64(double v, uint32_t position) {
    TetherIOClient::FunctionArg a;
    a.position = position;
    a.type = ValueType::F64;
    a.value.resize(8);
    std::memcpy(a.value.data(), &v, 8);
    return a;
}

TetherIOClient::FunctionArg argU8(uint8_t v, uint32_t position) {
    TetherIOClient::FunctionArg a;
    a.position = position;
    a.type = ValueType::U8;
    a.value = {v};
    return a;
}

void report(TetherIOClient& client, uint32_t returnSlot, const char* label,
            const ClientCallResult& r) {
    if (r.success && r.hasReturnValue) {
        // Return bytes are schema-encoded — decode against the slot the
        // function advertised in its catalog entry.
        const auto v = client.decodeValue(returnSlot, r.returnValue.data(),
                                          r.returnValue.size());
        if (v) {
            if (const auto s = asString(*v))
                std::printf("[client] %s -> \"%s\"\n", label, s->c_str());
            return;
        }
    }
    std::printf("[client] %s -> error %u: %s\n", label, r.errorCode,
                r.errorMessage.c_str());
}

} // namespace

int main(int argc, char** argv) {
    uint16_t port = 42782;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));

    // Schemas: F64/U8 param types + String return.
    SchemaGraph graph;
    auto scalar = [&](uint8_t tag, ValueType t, const char* name) {
        SchemaNode n;
        n.key = demoKey(tag); n.kind = SchemaKind::Scalar;
        n.scalarType = t; n.name = name;
        graph.nodes.push_back(std::move(n));
    };
    scalar(1, ValueType::F64, "demo.F64");
    scalar(2, ValueType::U8, "demo.U8");
    {
        SchemaNode n;
        n.key = demoKey(3); n.kind = SchemaKind::String;
        n.name = "demo.String";
        graph.nodes.push_back(std::move(n));
    }
    if (!resolveSchemaDigests(graph)) return 1;
    SchemaCatalog catalog;
    if (!catalog.install(graph, buildSchemaManifest(graph))) return 1;
    auto slotOf = [&](uint8_t tag) { return *catalog.slotForKey(demoKey(tag)); };
    auto refOf = [&](uint8_t tag) {
        return SchemaRef{demoKey(tag),
                         computeSchemaDigest(*graph.find(demoKey(tag)))};
    };

    Registry registry;
    FunctionEntry divide;
    divide.id = 1;
    divide.name = "math.divide";
    divide.group = "math";
    divide.description = "Returns a/b formatted to `precision` decimals";
    {
        FunctionParameter a;
        a.name = "a"; a.description = "numerator";
        a.type = ValueType::F64; a.schema = refOf(1); a.schemaSlot = slotOf(1);
        FunctionParameter b;
        b.name = "b"; b.description = "denominator";
        b.type = ValueType::F64; b.schema = refOf(1); b.schemaSlot = slotOf(1);
        FunctionParameter prec;
        prec.name = "precision"; prec.description = "decimals";
        prec.type = ValueType::U8; prec.schema = refOf(2); prec.schemaSlot = slotOf(2);
        prec.optional = true; prec.hasDefault = true;
        prec.defaultValue = {2};  // U8 2 — the server materializes it when omitted
        divide.parameters = {a, b, prec};
    }
    divide.returnValue.present = true;
    divide.returnValue.name = "text";
    divide.returnValue.type = ValueType::String;
    divide.returnValue.schema = refOf(3);
    divide.returnValue.schemaSlot = slotOf(3);
    const SchemaNode* strNode = graph.find(demoKey(3));
    divide.callback = [&graph, strNode](
                          const std::vector<FunctionArgument>& args) {
        FunctionCallResult r;
        double a = 0, b = 0;
        uint8_t precision = 2;
        for (const auto& arg : args) {
            if (!arg.provided) continue;
            if (arg.position == 0 && arg.value.size() == 8)
                std::memcpy(&a, arg.value.data(), 8);
            else if (arg.position == 1 && arg.value.size() == 8)
                std::memcpy(&b, arg.value.data(), 8);
            else if (arg.position == 2 && !arg.value.empty())
                precision = arg.value[0];
        }
        if (b == 0.0) {
            r.errorMessage = "division by zero";
            return r;  // success=false is the wire error path
        }
        char buf[64];
        std::snprintf(buf, sizeof buf, "%.*f",
                      static_cast<int>(precision), a / b);
        // Declared-schema returns are validated against the installed
        // node — raw bytes would be rejected, encode via the codec.
        auto enc = encodeNative(graph, *strNode, nv::str(buf));
        if (!enc) {
            r.errorMessage = "encode: " + enc.error();
            return r;
        }
        r.success = true;
        r.returnValue = std::move(*enc);
        return r;
    };
    if (!registry.addFunction(divide)) return 1;

    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.schemaCatalog = &catalog;
    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, "127.0.0.1"),
                  sc);
    if (!server.start()) { std::fprintf(stderr, "server start failed\n"); return 1; }

    auto transport = tcpConnect("127.0.0.1", port, 2000);
    if (!transport) return 1;
    TetherIOClient client(
        std::make_unique<SlipFramedTransport>(std::move(transport)), 2000);

    const auto functions = client.listFunctions();
    if (!functions || functions->empty()) return 1;
    const ClientFunctionEntry& fn = (*functions)[0];
    const uint64_t divideId = fn.id;
    const uint32_t retSlot = fn.returnSchemaSlot;

    if (const auto r = client.callFunction(divideId,
                                         {argF64(22.0, 0), argF64(7.0, 1)}))
        report(client, retSlot, "divide(22, 7)", *r);  // precision defaults to 2
    if (const auto r = client.callFunction(
            divideId, {argF64(22.0, 0), argF64(7.0, 1), argU8(6, 2)}))
        report(client, retSlot, "divide(22, 7, 6)", *r);
    if (const auto r = client.callFunction(divideId,
                                         {argF64(1.0, 0), argF64(0.0, 1)}))
        report(client, retSlot, "divide(1, 0)", *r);   // the error path

    server.stop();
    return 0;
}
