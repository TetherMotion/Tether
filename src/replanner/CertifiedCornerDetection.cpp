/**
 * @file CertifiedCornerDetection.cpp
 * @brief Implementation of certified corner detection via CornerAnalyzer
 */

#include "tether/motion_replanner/CertifiedCornerDetection.hpp"

#include <algorithm>
#include <stdexcept>

namespace tether::motion::replanner {

std::string CertifiedJunction::kindString() const {
    switch (analysis.kind) {
        case CornerKind::Straight: return "Straight";
        case CornerKind::Corner:   return "Corner";
        case CornerKind::Cusp:     return "Cusp";
    }
    return "Unknown";
}

std::optional<CertifiedJunction> CertifiedCornerDetection::junctionAt(
    std::size_t pieceInIndex) const {
    auto it = std::lower_bound(
        junctions.begin(), junctions.end(), pieceInIndex,
        [](const CertifiedJunction& j, std::size_t idx) {
            return j.pieceInIndex < idx;
        });
    if (it != junctions.end() && it->pieceInIndex == pieceInIndex) {
        return *it;
    }
    return std::nullopt;
}

CertifiedCornerDetection detectCorners(
    const PiecewiseNurbsPath& path,
    double minAngleRad,
    double maxAngleRad) {

    if (path.numPieces() < 2) {
        throw std::invalid_argument(
            "detectCorners requires at least 2 pieces, got " +
            std::to_string(path.numPieces()));
    }

    CornerAnalyzer analyzer(minAngleRad, maxAngleRad);

    CertifiedCornerDetection result;
    result.junctions.reserve(path.numPieces() - 1);

    for (std::size_t i = 0; i + 1 < path.numPieces(); ++i) {
        const NurbsCurve& in = path.piece(i);
        const NurbsCurve& out = path.piece(i + 1);

        CertifiedJunction j;
        j.pieceInIndex = i;
        j.pieceOutIndex = i + 1;
        j.analysis = analyzer.analyze(in, out);

        switch (j.analysis.kind) {
            case CornerKind::Straight: ++result.straightCount; break;
            case CornerKind::Corner:   ++result.cornerCount;   break;
            case CornerKind::Cusp:     ++result.cuspCount;     break;
        }

        result.junctions.push_back(std::move(j));
    }

    return result;
}

} // namespace tether::motion::replanner
