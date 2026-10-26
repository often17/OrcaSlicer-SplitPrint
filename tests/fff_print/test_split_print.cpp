#include <catch2/catch_all.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCode/ToolOrdering.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/SplitPrint/Export.hpp"
#include "libslic3r/SplitPrint/Scheduler.hpp"
#include "libslic3r/Surface.hpp"

#include "test_helpers.hpp"
#include "test_utils.hpp"

#include <boost/filesystem/operations.hpp>

using namespace Slic3r;
using namespace Slic3r::Test;
using Catch::Matchers::WithinRel;

namespace {

// Two heads, a 40 mm cube split at its centre with a 10 mm no-go zone.
DynamicPrintConfig split_config(bool enabled)
{
    DynamicPrintConfig config = multifilament_config(2);
    config.set_deserialize_strict({
        {"layer_height", "0.2"},
        {"initial_layer_print_height", "0.2"},
        {"split_print_enable", enabled ? "1" : "0"},
        {"split_axis", "x"},
        {"split_position", "0"},
        {"split_nogo_width", "10"},
        {"split_interface_length_min", "1"},
        {"split_interface_length_max", "4"},
        // Two heads share Z: a lift by one would lift the other mid-line.
        {"z_hop", "0,0"},
        {"printable_area", "0x0,400x0,400x400,0x400"},
        {"split_gantry_min_gap", "60"},
        {"skirt_loops", "0"},
        {"brim_type", "no_brim"},
        // The heads run Klipper; timed layers rely on its motion planner's times.
        {"gcode_flavor", "klipper"},
    });
    return config;
}

// Area of every layer printed by each head (index 0 = extruder 1), summed over the object.
std::array<double, 2> area_by_head(const Print &print)
{
    std::array<double, 2> area{0., 0.};
    for (const Layer *layer : print.objects().front()->layers())
        for (const LayerRegion *layerm : layer->regions()) {
            const int filament = layerm->region().config().outer_wall_filament_id.value;
            if (filament >= 1 && filament <= 2)
                area[filament - 1] += Slic3r::area(to_expolygons(layerm->slices.surfaces));
        }
    return area;
}

bool every_layer_uses_both_heads(Print &print)
{
    ToolOrdering &to = const_cast<ToolOrdering &>(print.tool_ordering());
    for (const LayerTools &lt : to.layer_tools())
        if (lt.has_object && std::count_if(lt.extruders.begin(), lt.extruders.end(), [](unsigned e) { return e <= 1; }) != 2)
            return false;
    return !to.layer_tools().empty();
}

} // namespace

TEST_CASE("Split print gives each head its half of every layer", "[SplitPrint]")
{
    Print print;
    Model model;
    init_print({cube(40)}, print, model, split_config(true));
    print.process();

    CHECK(every_layer_uses_both_heads(print));
    // Split at the centre of a symmetric cube: both heads print the same area.
    const std::array<double, 2> area = area_by_head(print);
    REQUIRE(area[0] > 0.);
    CHECK_THAT(area[1], WithinRel(area[0], 0.05));
}

TEST_CASE("Split print off leaves the object to one head", "[SplitPrint]")
{
    Print print;
    Model model;
    init_print({cube(40)}, print, model, split_config(false));
    print.process();

    const std::array<double, 2> area = area_by_head(print);
    CHECK(area[0] > 0.);
    CHECK(area[1] == 0.);
}

TEST_CASE("Printed lines fill the no-go zone so the halves join", "[SplitPrint]")
{
    const char *interlock = GENERATE("both", "in_plane", "layers");
    INFO("interlock: " << interlock);
    DynamicPrintConfig config = split_config(true);
    config.set_deserialize_strict({{"split_interface_interlock", interlock}, {"split_interface_layers_per_block", "2"}});
    Print print;
    Model model;
    init_print({cube(40)}, print, model, config);
    print.process();

    const PrintObject &object = *print.objects().front();
    const Layer       &layer  = *object.layers()[object.layers().size() / 2];
    const BoundingBox  bbox   = get_extents(layer.lslices);
    // The 10 mm zone around the centre, clipped to the part.
    BoundingBox band(Point(bbox.center().x() - scaled<coord_t>(5.), bbox.min.y()), Point(bbox.center().x() + scaled<coord_t>(5.), bbox.max.y()));
    const ExPolygons zone = intersection_ex(layer.lslices, Polygons{band.polygon()});

    std::array<Polygons, 2> printed;
    for (const LayerRegion *layerm : layer.regions()) {
        const int filament = layerm->region().config().outer_wall_filament_id.value;
        if (filament < 1 || filament > 2)
            continue;
        append(printed[filament - 1], layerm->perimeters.polygons_covered_by_width());
        append(printed[filament - 1], layerm->fills.polygons_covered_by_width());
    }
    const double zone_area = area(zone);
    REQUIRE(zone_area > 0.);
    // Both heads print inside the zone, and together they leave no gap.
    CHECK(area(intersection_ex(zone, printed[0])) > 0.1 * zone_area);
    CHECK(area(intersection_ex(zone, printed[1])) > 0.1 * zone_area);
    // The two heads' lines touch along the whole depth of the part.
    const coord_t       touch = scaled<coord_t>(0.05);
    const BoundingBox   joint = get_extents(intersection(offset(printed[0], touch), offset(printed[1], touch)));
    REQUIRE(joint.defined);
    CHECK(joint.size().y() > 0.9 * bbox.size().y());
    if (std::string(interlock) != "layers") {
        Polygons both = printed[0];
        append(both, printed[1]);
        // Rows alternate across the whole zone, so it is all finger lines and far denser than
        // sparse infill (about 25% here); only small voids at finger tips stay empty.
        // With "layers" each head owns solid areas of the zone, which get normal infill.
        CHECK(area(intersection_ex(zone, union_(both))) > 0.9 * zone_area);
    }
}

namespace {

// Slices `mesh` centred on a 400 x 400 bed and exports it, keeping the two head streams.
void export_split(const TriangleMesh &mesh, std::array<std::vector<std::string>, 2> &heads, GCodeProcessorResult &result,
                  const DynamicPrintConfig &extra = {})
{
    DynamicPrintConfig config = split_config(true);
    config.apply(extra);
    Print print;
    Model model;
    init_print({mesh}, print, model, config);
    model.center_instances_around_point(Vec2d(200., 200.));
    DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
    full.apply(config);
    print.apply(model, full);
    print.process();
    ScopedTemporaryFile temp(".gcode");
    print.export_gcode(temp.string(), &result, nullptr);
    for (int h = 0; h < 2; ++h) {
        const std::string path = SplitPrint::head_gcode_path(temp.string(), h);
        std::ifstream     in(path, std::ios::binary);
        REQUIRE(in.good());
        for (std::string line; std::getline(in, line);)
            heads[h].push_back(line);
        in.close();
        boost::filesystem::remove(path);
    }
}

SplitPrint::ScheduleParams bed_params()
{
    SplitPrint::ScheduleParams p;
    p.axis    = 'X';
    p.gap     = 60.;
    p.bed_min = 0.;
    p.bed_max = 400.;
    return p;
}

bool is_tool_change(const std::string &line) { return line.size() >= 2 && line[0] == 'T' && std::isdigit((unsigned char) line[1]); }

} // namespace

TEST_CASE("Each head gets its own G-code that keeps the heads apart", "[SplitPrint]")
{
    // With stops in the middle of layers, safety does not depend on timing, so the files can be
    // checked here without move times.
    DynamicPrintConfig stops;
    stops.set_deserialize_strict({{"split_timed_layers", "0"}});
    std::array<std::vector<std::string>, 2> heads;
    GCodeProcessorResult                    result;
    export_split(cube(40), heads, result, stops);

    const std::vector<std::string> errors = SplitPrint::check_streams(heads, bed_params());
    INFO((errors.empty() ? std::string() : errors.front()));
    CHECK(errors.empty());
    for (int h = 0; h < 2; ++h) {
        // Each board has one extruder: no tool changes, and the head prints something.
        CHECK(std::none_of(heads[h].begin(), heads[h].end(), is_tool_change));
        CHECK(std::count_if(heads[h].begin(), heads[h].end(), [](const std::string &l) { return l.rfind("G1 X", 0) == 0 && l.find(" E") != std::string::npos; }) > 100);
    }
    CHECK(SplitPrint::sync_lines(heads[0]).size() == SplitPrint::sync_lines(heads[1]).size());
}

TEST_CASE("Split preview shows both heads on one timeline", "[SplitPrint]")
{
    std::array<std::vector<std::string>, 2> heads;
    GCodeProcessorResult                    result;
    export_split(cube(40), heads, result);

    REQUIRE(result.split_print);
    CHECK(result.split_one_head_time > 0.f);
    bool head[2] = {false, false};
    for (const GCodeProcessorResult::MoveVertex &m : result.moves)
        if (m.type == EMoveType::Extrude && m.extruder_id < 2)
            head[m.extruder_id] = true;
    CHECK(head[0]);
    CHECK(head[1]);
}

TEST_CASE("A long part prints faster with two heads", "[SplitPrint]")
{
    // 300 mm long: most of each half is more than the 60 mm gap away from the other head.
    std::array<std::vector<std::string>, 2> heads;
    GCodeProcessorResult                    result;
    export_split(make_cube(300., 30., 1.), heads, result);

    const float two_heads = result.print_statistics.modes[0].time;
    REQUIRE(result.split_one_head_time > 0.f);
    // Ideal is half; the middle and the layer changes cost some of it.
    CHECK(two_heads < 0.7f * result.split_one_head_time);
}

TEST_CASE("Old layer shift setting loads as an interlock direction", "[SplitPrint]")
{
    const bool on = GENERATE(true, false);
    DynamicPrintConfig config;
    config.set_deserialize_strict({{"split_interface_layer_shift", on ? "1" : "0"}});
    REQUIRE(config.has("split_interface_interlock"));
    CHECK(config.opt_serialize("split_interface_interlock") == (on ? "both" : "in_plane"));
}

TEST_CASE("Turning split print off on an existing print rebuilds its regions", "[SplitPrint]")
{
    Print print;
    Model model;
    init_print({cube(40)}, print, model, split_config(true));
    print.process();
    REQUIRE(area_by_head(print)[1] > 0.);

    print.apply(model, split_config(false));
    print.process();
    CHECK(area_by_head(print)[1] == 0.);
}

TEST_CASE("Slicing again over the old files writes both heads anew", "[SplitPrint]")
{
    // The app slices each time to the same temporary path, so the files of the last slice are
    // still there: a 2 mm cube is 10 layers at 0.2 mm, the 4 mm one that replaces it 20.
    DynamicPrintConfig config = split_config(true);
    DynamicPrintConfig full   = DynamicPrintConfig::full_print_config();
    full.apply(config);
    ScopedTemporaryFile temp(".gcode");
    Print               print;
    for (const double height : {2., 4.}) {
        Model model;
        init_print({make_cube(40., 40., height)}, print, model, config);
        model.center_instances_around_point(Vec2d(200., 200.));
        print.apply(model, full);
        print.process();
        GCodeProcessorResult result;
        REQUIRE_NOTHROW(print.export_gcode(temp.string(), &result, nullptr));
        for (int h = 0; h < 2; ++h) {
            std::vector<std::string> lines;
            std::ifstream            in(SplitPrint::head_gcode_path(temp.string(), h), std::ios::binary);
            for (std::string line; std::getline(in, line);)
                lines.push_back(line);
            const size_t layer_syncs = std::count_if(lines.begin(), lines.end(), [](const std::string &l) {
                return l.rfind("; SPLITPRINT_SYNC L", 0) == 0 && l.find_first_of(".z", 19) == std::string::npos;
            });
            INFO("height " << height << " head " << h);
            CHECK(layer_syncs == size_t(std::lround(height / 0.2)));
        }
    }
    for (int h = 0; h < 2; ++h)
        boost::filesystem::remove(SplitPrint::head_gcode_path(temp.string(), h));
}

TEST_CASE("Fingers two lines wide do not need a retraction each", "[SplitPrint]")
{
    // A 200 mm long joint on 5 sparse layers. Fingers one line wide are separate short lines,
    // each with its own retraction; two lines wide, they are part of the wall around them.
    auto head_retractions = [](const char *finger_width) {
        DynamicPrintConfig config = split_config(true);
        config.set_deserialize_strict({{"wall_generator", "arachne"}, {"retraction_length", "0.8,0.8"},
                                       {"split_interface_finger_width", finger_width},
                                       {"bottom_shell_layers", "0"}, {"top_shell_layers", "0"}});
        Print print;
        Model model;
        init_print({make_cube(100., 200., 1.)}, print, model, config);
        model.center_instances_around_point(Vec2d(200., 200.));
        DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
        full.apply(config);
        print.apply(model, full);
        print.process();
        ScopedTemporaryFile  temp(".gcode");
        GCodeProcessorResult result;
        print.export_gcode(temp.string(), &result, nullptr);
        size_t retractions = 0;
        for (int h = 0; h < 2; ++h) {
            const std::string path = SplitPrint::head_gcode_path(temp.string(), h);
            {
                std::ifstream in(path, std::ios::binary);
                for (std::string line; std::getline(in, line);)
                    if (line.rfind("G1 E-", 0) == 0)
                        ++retractions;
            }
            boost::filesystem::remove(path);
        }
        return retractions;
    };
    const size_t one_line  = head_retractions("1");
    const size_t two_lines = head_retractions("2");
    // About 110 fingers per head on each of the 5 layers with one-line fingers.
    CHECK(one_line > 1000);
    CHECK(two_lines * 4 < one_line);
}

namespace {

// Length printed per feature (as named by the role tag) in a G-code file, and the X range of each feature.
struct FeatureRun
{
    double length = 0.;
    double x_min  = 1e9;
    double x_max  = -1e9;
};
std::map<std::string, FeatureRun> feature_runs(const std::string &path)
{
    std::map<std::string, FeatureRun> runs;
    std::ifstream                     in(path, std::ios::binary);
    std::string                       feature;
    double                            x = 0., y = 0.;
    auto value = [](const std::string &line, char axis, double &v) {
        const size_t k = line.find(std::string(" ") + axis);
        if (k == std::string::npos)
            return false;
        v = std::stod(line.substr(k + 2));
        return true;
    };
    for (std::string line; std::getline(in, line);) {
        // The role tag is ";TYPE:" or "; FEATURE: ", depending on the printer.
        const size_t type = line.rfind(";TYPE:", 0) == 0 ? 6 : line.rfind("; FEATURE: ", 0) == 0 ? 11 : 0;
        if (type > 0) {
            feature = line.substr(type);
            while (!feature.empty() && std::isspace((unsigned char) feature.back()))
                feature.pop_back();
            continue;
        }
        if (line.rfind("G1 ", 0) != 0 && line.rfind("G0 ", 0) != 0)
            continue;
        const std::string code = line.substr(0, line.find(';'));
        double nx = x, ny = y, e = 0.;
        value(code, 'X', nx);
        value(code, 'Y', ny);
        if (value(code, 'E', e) && e > 0. && (nx != x || ny != y)) {
            FeatureRun &r = runs[feature];
            r.length += std::hypot(nx - x, ny - y);
            r.x_min = std::min({r.x_min, x, nx});
            r.x_max = std::max({r.x_max, x, nx});
        }
        x = nx;
        y = ny;
    }
    return runs;
}

} // namespace

TEST_CASE("Each head prints the brim and skirt on its own side", "[SplitPrint]")
{
    // A 40 mm cube centred at X = 200, split at its centre, with a 5 mm brim and one skirt loop.
    DynamicPrintConfig config = split_config(true);
    config.set_deserialize_strict({{"brim_type", "outer_only"}, {"brim_width", "5"}, {"skirt_loops", "1"}, {"skirt_distance", "3"}});
    Print print;
    Model model;
    init_print({cube(40)}, print, model, config);
    model.center_instances_around_point(Vec2d(200., 200.));
    DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
    full.apply(config);
    print.apply(model, full);
    print.process();
    ScopedTemporaryFile  temp(".gcode");
    GCodeProcessorResult result;
    print.export_gcode(temp.string(), &result, nullptr);

    const std::map<std::string, FeatureRun> combined = feature_runs(temp.string());
    std::array<std::map<std::string, FeatureRun>, 2> heads;
    for (int h = 0; h < 2; ++h) {
        const std::string path = SplitPrint::head_gcode_path(temp.string(), h);
        heads[h] = feature_runs(path);
        boost::filesystem::remove(path);
    }
    for (const char *feature : {"Brim", "Skirt"}) {
        INFO(feature);
        REQUIRE(combined.count(feature) == 1);
        REQUIRE(heads[0].count(feature) == 1);
        REQUIRE(heads[1].count(feature) == 1);
        // Shared, not printed twice: together the heads print what one head prints alone.
        CHECK_THAT(heads[0].at(feature).length + heads[1].at(feature).length, WithinRel(combined.at(feature).length, 0.03));
        CHECK(heads[0].at(feature).x_max <= 200.01);
        CHECK(heads[1].at(feature).x_min >= 199.99);
    }
}

namespace {

bool is_mid_layer_sync(const std::string &l) { return l.rfind("; SPLITPRINT_SYNC L", 0) == 0 && l.find('.', 19) != std::string::npos; }
size_t count_starting(const std::vector<std::string> &lines, const char *prefix)
{
    return size_t(std::count_if(lines.begin(), lines.end(), [prefix](const std::string &l) { return l.rfind(prefix, 0) == 0; }));
}

} // namespace

TEST_CASE("Each layer prints without a stop in the middle", "[SplitPrint]")
{
    // A 40 mm cube is narrower than the 60 mm gap: the heads print each layer one after the
    // other. A 300 mm bar is wide enough for both to work at once. Both are 1 mm high: 5 layers.
    const bool wide = GENERATE(false, true);
    std::array<std::vector<std::string>, 2> heads;
    GCodeProcessorResult                    result;
    export_split(wide ? make_cube(300., 30., 1.) : make_cube(40., 40., 1.), heads, result);

    INFO((wide ? "300 mm bar" : "40 mm cube"));
    CHECK(result.split_stopped_layers == 0);
    for (int h = 0; h < 2; ++h) {
        INFO("head " << h);
        CHECK(std::none_of(heads[h].begin(), heads[h].end(), is_mid_layer_sync));
        CHECK(count_starting(heads[h], "; SPLITPRINT_TIMED ") == 5);
    }
}

TEST_CASE("Two heads save time on a long part without stopping mid-layer", "[SplitPrint]")
{
    std::array<std::vector<std::string>, 2> heads;
    GCodeProcessorResult                    result;
    export_split(make_cube(300., 30., 1.), heads, result);
    REQUIRE(result.split_one_head_time > 0.f);
    CHECK(result.split_stopped_layers == 0);
    CHECK(result.print_statistics.modes[0].time < 0.8f * result.split_one_head_time);
}

TEST_CASE("Turning timed layers off brings back stops in the middle of layers", "[SplitPrint]")
{
    DynamicPrintConfig stops;
    stops.set_deserialize_strict({{"split_timed_layers", "0"}});
    std::array<std::vector<std::string>, 2> heads;
    GCodeProcessorResult                    result;
    export_split(make_cube(40., 40., 1.), heads, result, stops);
    for (int h = 0; h < 2; ++h) {
        CHECK(count_starting(heads[h], "; SPLITPRINT_TIMED ") == 0);
        CHECK(std::any_of(heads[h].begin(), heads[h].end(), is_mid_layer_sync));
    }
}

TEST_CASE("A preview over its move budget keeps every n-th layer and the whole print time", "[SplitPrint]")
{
    DynamicPrintConfig config = split_config(true);
    Print              print;
    Model              model;
    init_print({cube(40)}, print, model, config);
    model.center_instances_around_point(Vec2d(200., 200.));
    DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
    full.apply(config);
    print.apply(model, full);
    print.process();
    ScopedTemporaryFile                temp(".gcode");
    std::array<std::vector<double>, 2> times;
    for (int h = 0; h < 2; ++h) {
        GCode gcode;
        times[h] = SplitPrint::export_head_stream(print, gcode, SplitPrint::head_gcode_path(temp.string(), h), h);
    }
    const std::shared_ptr<SplitPrint::HeadsPreview> heads = SplitPrint::schedule_head_streams(print, temp.string(), std::move(times), true);
    // Both previews come from the same scheduled files: one keeps every move, one an eighth of them.
    // Layer 0 is the start G-code, which is always shown.
    GCodeProcessorResult all, thinned;
    SplitPrint::merge_head_preview(print, heads, &all, size_t(1) << 30);
    SplitPrint::merge_head_preview(print, heads, &thinned, all.moves.size() / 8);
    for (int h = 0; h < 2; ++h)
        boost::filesystem::remove(SplitPrint::head_gcode_path(temp.string(), h));

    CHECK(all.split_preview_layer_step == 1);
    const int step = thinned.split_preview_layer_step;
    REQUIRE(step > 1);
    CHECK(thinned.moves.size() < all.moves.size() / 2);
    for (const GCodeProcessorResult::MoveVertex &m : thinned.moves)
        if (m.type == EMoveType::Extrude && m.layer_id > 0 && (m.layer_id - 1) % step != 0) {
            FAIL("an extrusion of layer " << m.layer_id << " is shown");
            break;
        }
    // The layers left out still take their time.
    const float time = all.print_statistics.modes[0].time;
    REQUIRE(time > 0.f);
    CHECK_THAT(thinned.print_statistics.modes[0].time, WithinRel(time, 1e-4f));
}
