#include "tether/io/SchemaCatalog.hpp"

#include <gtest/gtest.h>

namespace tether::io {
namespace {

SchemaKey catalogKey(uint8_t value) {
    SchemaKey result{};
    result[0] = value;
    return result;
}

SchemaRef catalogRef(uint8_t value) { return SchemaRef{catalogKey(value), {}}; }

SchemaRef catalogRefFor(const SchemaNode& node) {
    return SchemaRef{node.key, computeSchemaDigest(node)};
}

TEST(IOSchemaCatalog, AssignsStableSortedSlotsWithinAnEpoch) {
    SchemaNode first;
    first.key = catalogKey(1);
    first.revision = 3;
    SchemaNode second;
    second.key = catalogKey(2);
    second.revision = 4;
    SchemaGraph graph{{first, second}};

    SchemaCatalog catalog;
    const auto firstRef = catalogRefFor(first);
    const auto secondRef = catalogRefFor(second);
    ASSERT_TRUE(catalog.install(graph, {{secondRef, 4}, {firstRef, 3}}));
    const auto epoch = catalog.epoch();
    ASSERT_TRUE(epoch != 0);
    ASSERT_EQ(catalog.slotFor(firstRef), 0U);
    ASSERT_EQ(catalog.slotFor(secondRef), 1U);
    EXPECT_EQ(catalog.resolve(epoch, 0)->key, catalogKey(1));
    EXPECT_EQ(catalog.resolve(epoch, 1)->key, catalogKey(2));
    EXPECT_EQ(catalog.resolve(epoch + 1, 0), nullptr);
}

TEST(IOSchemaCatalog, RejectsUnknownOrDuplicateManifestEntries) {
    SchemaNode node;
    node.key = catalogKey(1);
    SchemaGraph graph{{node}};
    SchemaCatalog catalog;

    EXPECT_FALSE(catalog.install(graph, {{catalogRef(9), 1}}));
    EXPECT_FALSE(catalog.install(graph, {{catalogRefFor(node), 1}, {catalogRefFor(node), 1}}));
    EXPECT_FALSE(catalog.install(graph, {{catalogRefFor(node), 2}}));
}

TEST(IOSchemaCatalog, RejectsDigestMismatch) {
    SchemaNode node;
    node.key = catalogKey(1);
    SchemaGraph graph{{node}};
    auto ref = catalogRefFor(node);
    ref.digest[0] ^= 0xff;

    SchemaCatalog catalog;
    EXPECT_FALSE(catalog.install(graph, {{ref, node.revision}}));
}

} // namespace
} // namespace tether::io
