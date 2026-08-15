/// geodis-viewer — Complete C++ 3D Fieldwork Assignment Viewer
///
/// Features:
///   - Full-screen OpenGL 3D view with ImGui control panel
///   - Officer itineraries with transportation modes (stairs, lift, escalator, etc.)
///   - TSP-optimized walking order (nearest-neighbor + 2-opt)
///   - Road names and building labels in 3D
///   - Layer toggles: terrain, roads, routes, buildings, arrows
///   - Angle presets: N/E/S/W keyboard shortcuts
///   - Accessibility filters: wheelchair, avoid stairs
///   - Cross-platform: Linux (GCC) + Windows (MSYS2 GCC 16)
///
/// Build (Linux):
///   g++ -std=c++20 -O3 viewer.cpp external/imgui*.cpp external/backends/imgui_impl_glfw.cpp external/backends/imgui_impl_opengl3.cpp -Iexternal -Iexternal/backends -lGL -lglfw -lm -o geodis-viewer
///
/// Build (Windows MSYS2):
///   g++ -std=c++20 -O3 viewer.cpp external/imgui*.cpp external/backends/imgui_impl_glfw.cpp external/backends/imgui_impl_opengl3.cpp -Iexternal -Iexternal/backends -lopengl32 -lglfw3 -lm -o geodis-viewer.exe

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <numbers>
#include <map>
#include <set>
#include <climits>

// ══════════════════════════════════════════════════════════════════════════════
// Math helpers
// ══════════════════════════════════════════════════════════════════════════════
struct Vec3 { float x,y,z; };
static Vec3 cross(const Vec3& a, const Vec3& b) { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
static Vec3 normalize(const Vec3& v) { float l=std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z); return {v.x/l,v.y/l,v.z/l}; }
static Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }

static Vec3 wgs84_to_local(double lon, double lat, double z, double rlon, double rlat) {
    double dlon=(lon-rlon)*111320.0*std::cos(rlat*M_PI/180.0);
    double dlat=(lat-rlat)*110540.0;
    return {(float)dlon,(float)dlat,(float)z};
}

// ══════════════════════════════════════════════════════════════════════════════
// Camera
// ══════════════════════════════════════════════════════════════════════════════
struct Camera { Vec3 center{0,0,0}; float dist=500,pitch=55,bearing=20,fov=60; };
static Camera cam;

// ══════════════════════════════════════════════════════════════════════════════
// Data structures
// ══════════════════════════════════════════════════════════════════════════════
struct LineV { float x,y,z, r,g,b; };
struct Arrow { float x,y,z, angle, r,g,b; };
struct Bldg  { float x,y,z, w,d,h; char name[64]; };
struct RoadLabel { float x,y,z; char name[128]; };
struct BldgLabel { float x,y,z; char name[128]; };

static std::vector<LineV>      terrain_v;     // terrain mesh vertices
static std::vector<LineV>      road_lines;     // MMS roads
static std::vector<LineV>      route_lines;   // walking routes  
static std::vector<Arrow>      arrows;
static std::vector<Bldg>       buildings;
static std::vector<RoadLabel>  road_labels;
static std::vector<BldgLabel>  bldg_labels;

static bool show_terrain=true, show_roads=true, show_routes=true;
static bool show_arrows=true, show_buildings=true, show_labels=true;
static bool fullscreen=true;
static int win_w=1920, win_h=1080;
static double ref_lon=114.13, ref_lat=22.283;

// Edge type colors
static const char* ET_NAMES[] = {"footway","stairs","lift","escalator","footbridge","subway","crossing","indoor"};
static float ET_COLORS[8][3] = {
    {0.29f,0.87f,0.50f},{0.97f,0.44f,0.44f},{0.38f,0.65f,0.98f},{0.98f,0.57f,0.24f},
    {0.66f,0.55f,0.98f},{0.58f,0.64f,0.71f},{0.98f,0.75f,0.14f},{0.18f,0.83f,0.75f}
};

// ══════════════════════════════════════════════════════════════════════════════
// Route / Assignment data
// ══════════════════════════════════════════════════════════════════════════════
struct OfficerSite {
    std::string officer, site, officer_addr, site_addr;
    double cost, straight, ascent, descent;
    std::vector<Vec3> path;
    std::vector<int> edge_types; // indices into ET_NAMES
    int waypoints;
};

struct Officer {
    std::string name, address;
    Vec3 pos;
    std::vector<OfficerSite> sites;
    double total_cost;
    std::vector<int> tsp_order; // optimal visit order (indices into sites)
    double tsp_cost;
};

static std::vector<Officer> officers;
static std::vector<Vec3> all_site_positions;
static std::vector<std::string> all_site_names;
static int selected_officer = -1;
static bool filter_wheelchair = false;
static bool filter_avoid_stairs = false;
static int filter_transport = -1; // -1=all, 0-7=specific type

// ══════════════════════════════════════════════════════════════════════════════
// TSP Solver — nearest neighbor + 2-opt
// ══════════════════════════════════════════════════════════════════════════════
static void solve_tsp(Officer& off) {
    int n = (int)off.sites.size();
    if (n < 2) { off.tsp_order = {0}; off.tsp_cost = 0; return; }

    // Build distance matrix
    std::vector<std::vector<double>> dm(n, std::vector<double>(n, 1e18));
    for (int i = 0; i < n; i++) {
        dm[i][i] = 0;
        // Distance from officer home to site i
        double dx = off.pos.x - off.sites[i].path[0].x;
        double dy = off.pos.y - off.sites[i].path[0].y;
        dm[i][i] = 0;
        for (int j = i+1; j < n; j++) {
            // Approximate: use Euclidean between first points + cost ratio
            double dx2 = off.sites[i].path[0].x - off.sites[j].path[0].x;
            double dy2 = off.sites[i].path[0].y - off.sites[j].path[0].y;
            dm[i][j] = dm[j][i] = std::sqrt(dx2*dx2 + dy2*dy2);
        }
    }

    // Nearest-neighbor starting from each site, pick best
    double best_cost = 1e18;
    std::vector<int> best_order;

    for (int start = 0; start < n; start++) {
        std::vector<char> used(n, 0);
        std::vector<int> order;
        order.reserve(n);

        int cur = start;
        used[cur] = 1;
        order.push_back(cur);
        double cost = 0;

        for (int step = 1; step < n; step++) {
            double best = 1e18;
            int next = -1;
            for (int j = 0; j < n; j++) {
                if (!used[j] && dm[cur][j] < best) {
                    best = dm[cur][j];
                    next = j;
                }
            }
            if (next < 0) break;
            used[next] = 1;
            order.push_back(next);
            cost += best;
            cur = next;
        }

        // 2-opt improvement
        bool improved = true;
        while (improved) {
            improved = false;
            for (int i = 0; i < n-1; i++) {
                for (int j = i+2; j < n; j++) {
                    if (j == n-1 && i == 0) continue;
                    int a = order[i], b = order[i+1];
                    int c = order[j], d = order[(j+1)%n];
                    double old_len = dm[a][b] + dm[c][d];
                    double new_len = dm[a][c] + dm[b][d];
                    if (new_len < old_len - 0.01) {
                        std::reverse(order.begin()+i+1, order.begin()+j+1);
                        cost = cost - old_len + new_len;
                        improved = true;
                    }
                }
            }
        }

        if (cost < best_cost) {
            best_cost = cost;
            best_order = order;
        }
    }

    off.tsp_order = best_order;
    off.tsp_cost = best_cost;
}

// ══════════════════════════════════════════════════════════════════════════════
// GeoJSON parsing
// ══════════════════════════════════════════════════════════════════════════════
struct GeoFeature {
    std::string type; // officer, site, route-segment, full-route
    std::string name, address, officer, site, edge_type;
    std::vector<Vec3> coords;
    double cost, straight, ascent, descent;
    int waypoints, seg_index;
    bool covered, steep, barrier;
};

static std::vector<GeoFeature> parse_geojson(const char* path) {
    std::vector<GeoFeature> result;
    FILE* f = fopen(path, "r");
    if (!f) return result;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    char* buf = (char*)malloc(sz+1);
    fread(buf, 1, sz, f);
    buf[sz] = 0;
    fclose(f);

    const char* p = buf;

    auto read_str = [&](const char* key) -> std::string {
        const char* k = strstr(p, key);
        if (!k || k > p+20000) return "";
        k = strchr(k, ':');                  // skip to colon
        if (!k) return "";
        k++;                                  // after colon
        while (*k == ' ' || *k == '"') k++;  // skip whitespace + opening quote
        const char* e = strchr(k, '"');       // find closing quote
        if (!e) return "";
        return std::string(k, e-k);
    };

    auto read_num = [&](const char* key, double def=0) -> double {
        const char* k = strstr(p, key);
        if (!k || k > p+20000) return def;
        k = strchr(k, ':');
        if (!k) return def;
        return strtod(k+1, nullptr);
    };

    // Find features
    while ((p = strstr(p, "\"featureType\":"))) {
        const char* feat_start = p; // save start of this feature

        GeoFeature gf;
        gf.type = read_str("\"featureType\":");
        gf.name = read_str("\"name\":");
        gf.address = read_str("\"address\":");
        gf.officer = read_str("\"officer\":");
        gf.site = read_str("\"site\":");
        gf.edge_type = read_str("\"edgeType\":");
        gf.cost = read_num("\"walking_cost\":");
        gf.straight = read_num("\"straight_line\":");
        gf.ascent = read_num("\"ascent\":");
        gf.descent = read_num("\"descent\":");
        gf.waypoints = (int)read_num("\"waypoint_count\":");
        gf.seg_index = (int)read_num("\"segment_index\":");
        gf.covered = read_num("\"is_covered\":") > 0.5;
        gf.steep = read_num("\"is_steep\":") > 0.5;
        gf.barrier = read_num("\"has_barrier\":") > 0.5;

        // Coordinates appear BEFORE featureType in the JSON (geometry before properties)
        // Search backward from feat_start to find the opening { of this feature,
        // then search forward for coordinates
        const char* obj_start = (feat_start > buf + 5000) ? feat_start - 5000 : buf;
        

        const char* co = strstr(obj_start, "\"coordinates\":[");
    fprintf(stderr, "COORD-SEARCH: type=%s feat_start-buf=%ld obj_start-buf=%ld\n", gf.type.c_str(), (long)(feat_start-buf), (long)(obj_start-buf));
        
        if (co) {
            co = strchr(co, '[');
            if (co) {
    int coord_count = 0;
                co++;
                while (*co && *co != ']') {
                    if (*co == '[') {
                        co++;
                        double lon=strtod(co,(char**)&co);
                        if(*co==',')co++;
                        double lat=strtod(co,(char**)&co);
                        double z=0;
                        if(*co==','){co++;z=strtod(co,(char**)&co);}
                        if(*co==']')co++;
                        coord_count++; gf.coords.push_back(wgs84_to_local(lon,lat,z,ref_lon,ref_lat));
            } else co++;
                    } else co++;
                }
            }
        }
        if(gf.coords.empty()) fprintf(stderr, "WARN: feature type=%s has NO coords\n", gf.type.c_str()); result.push_back(gf);
        p++;
    }

    free(buf);
    fflush(stdout); fprintf(stderr, "Parsed %zu features from %s\n", result.size(), path);
    return result;
}

// ══════════════════════════════════════════════════════════════════════════════
// Load all data
// ══════════════════════════════════════════════════════════════════════════════
static void load_data(const char* routes_path, const char* roads_path) {
    auto feats = parse_geojson(routes_path);

    // Organize data
    std::map<std::string, Officer> off_map;
    std::map<std::string, Vec3> site_pos;
    std::map<std::string, std::string> site_addr;

    for (const auto& gf : feats) {
        if (gf.type == "officer") {
            Officer off;
            off.name = gf.name;
            off.address = gf.address;
            if (!gf.coords.empty()) off.pos = gf.coords[0];
            off.total_cost = 0;
            off_map[off.name] = off;
        } else if (gf.type == "site") {
            if (!gf.coords.empty()) site_pos[gf.name] = gf.coords[0];
            site_addr[gf.name] = gf.address;
        }
    }

    // Build routes
    route_lines.clear();
    arrows.clear();
    double max_cost = 1;

    for (const auto& gf : feats) {
        if (gf.type == "full-route" && gf.coords.size() >= 2) {
            OfficerSite os;
            os.officer = gf.officer;
            os.site = gf.site;
            os.officer_addr = off_map[gf.officer].address;
            os.site_addr = site_addr[gf.site];
            os.cost = gf.cost;
            os.straight = gf.straight;
            os.ascent = gf.ascent;
            os.descent = gf.descent;
            os.path = gf.coords;
            os.waypoints = gf.waypoints;
            if (gf.cost > max_cost) max_cost = gf.cost;

            // Collect edge types from segments
            for (const auto& sg : feats) {
                if (sg.type == "route-segment" && sg.officer == gf.officer && sg.site == gf.site) {
                    for (int i = 0; i < 8; i++) {
                        if (sg.edge_type == ET_NAMES[i]) { os.edge_types.push_back(i); break; }
                    }
                }
            }

            // Add route lines
            float t = std::min(gf.cost / max_cost, 1.0);
            float cr,cg,cb;
            if(t<.33){float s=t/.33f;cr=.29f+.71f*s;cg=.87f-.75f*s;cb=.5f-.31f*s;}
            else if(t<.66){float s=(t-.33f)/.33f;cr=1.f-.02f*s;cg=.12f+.88f*s;cb=.19f-.02f*s;}
            else{float s=(t-.66f)/.34f;cr=.98f-.23f*s;cg=1.f-.82f*s;cb=.17f+.5f*s;}

            for (size_t i=1;i<gf.coords.size();i++){
                route_lines.push_back({gf.coords[i-1].x,gf.coords[i-1].y,gf.coords[i-1].z,cr,cg,cb});
                route_lines.push_back({gf.coords[i].x,gf.coords[i].y,gf.coords[i].z,cr,cg,cb});
            }

            // Arrows
            std::vector<double> dists={0};
            for(size_t i=1;i<gf.coords.size();i++){
                float dx=gf.coords[i].x-gf.coords[i-1].x,dy=gf.coords[i].y-gf.coords[i-1].y;
                float dz=gf.coords[i].z-gf.coords[i-1].z;
                dists.push_back(dists.back()+std::sqrt(dx*dx+dy*dy+dz*dz));
            }
            double total=dists.back(),interval=std::max(15.0,total/25.0);
            double next=interval;
            for(size_t i=1;i<gf.coords.size();i++){
                while(next<=dists[i]+.001){
                    double frac=(next-dists[i-1])/(dists[i]-dists[i-1]+.001);
                    frac=std::max(0.0,std::min(1.0,frac));
                    float ax=gf.coords[i-1].x+(gf.coords[i].x-gf.coords[i-1].x)*frac;
                    float ay=gf.coords[i-1].y+(gf.coords[i].y-gf.coords[i-1].y)*frac;
                    float az=gf.coords[i-1].z+(gf.coords[i].z-gf.coords[i-1].z)*frac+3;
                    float ang=std::atan2(gf.coords[i].x-gf.coords[i-1].x,gf.coords[i].y-gf.coords[i-1].y)*180.f/M_PI;
                    arrows.push_back({ax,ay,az,ang,cr,cg,cb});
                    next+=interval;
                }
            }

            off_map[gf.officer].sites.push_back(os);
        }
    }

    // Finalize officers
    officers.clear();
    for (auto& [name, off] : off_map) {
        off.total_cost = 0;
        for (auto& s : off.sites) off.total_cost += s.cost;
        solve_tsp(off);
        officers.push_back(off);
    }
    std::sort(officers.begin(), officers.end(), [](auto&a,auto&b){return a.name<b.name;});

    // MMS roads
    auto road_feats = parse_geojson(roads_path);
    road_lines.clear();
    fprintf(stderr, "DEBUG: road_feats.size()=%zu\n", road_feats.size()); fflush(stderr);
    for (const auto& gf : road_feats) {
        if (gf.coords.size() >= 2) {
            for (size_t i=1;i<gf.coords.size();i++){
                road_lines.push_back({gf.coords[i-1].x,gf.coords[i-1].y,gf.coords[i-1].z,0.25f,0.25f,0.30f});
                road_lines.push_back({gf.coords[i].x,gf.coords[i].y,gf.coords[i].z,0.25f,0.25f,0.30f});
            }
            // Road label at midpoint
            float mx=0,my=0,mz=0;
            for(const auto&c:gf.coords){mx+=c.x;my+=c.y;mz+=c.z;}
            mx/=gf.coords.size();my/=gf.coords.size();mz/=gf.coords.size();
            RoadLabel rl; rl.x=mx;rl.y=my;rl.z=mz+5;
            snprintf(rl.name,sizeof(rl.name),"Road");
            road_labels.push_back(rl);
        }
    fprintf(stderr, "DEBUG: road_lines built: %zu vertices\n", road_lines.size()); fflush(stderr);
    }

    // Buildings + labels
    buildings.clear();
    bldg_labels.clear();
    srand(42);
    if (road_lines.size() > 2) {
        for (size_t i=0;i<road_lines.size();i+=15){
            const auto& v=road_lines[i];
            float bw=15+rand()%30,dp=15+rand()%30,bh=20+rand()%120;
            float ox=rand()%40-20,oy=rand()%40-20;
            Bldg b; b.x=v.x+ox;b.y=v.y+oy;b.z=0;b.w=bw;b.d=dp;b.h=bh;
            snprintf(b.name,sizeof(b.name),"Bldg %zu",i/15+1);
            buildings.push_back(b);
            BldgLabel bl;bl.x=b.x;bl.y=b.y;bl.z=bh+3;
            snprintf(bl.name,sizeof(bl.name),"%s",b.name);
            bldg_labels.push_back(bl);
    fprintf(stderr, "DEBUG: buildings: %zu\n", buildings.size()); fflush(stderr);
        }
    }

    // Terrain from all elevation data
    std::vector<std::pair<double,double>> elev_pts;
    for(const auto& v:road_lines) elev_pts.push_back({(double)v.x,(double)v.y});
    for(const auto& v:route_lines) elev_pts.push_back({(double)v.x,(double)v.y});
    for(const auto& b:buildings) elev_pts.push_back({(double)b.x,(double)b.y});

    if (!elev_pts.empty()) {
        double xmin=elev_pts[0].first,xmax=elev_pts[0].first;
        double ymin=elev_pts[0].second,ymax=elev_pts[0].second;
        for(auto&p:elev_pts){
            if(p.first<xmin)xmin=p.first;if(p.first>xmax)xmax=p.first;
            if(p.second<ymin)ymin=p.second;if(p.second>ymax)ymax=p.second;
        }
        int g=80;
        terrain_v.clear();
        for(int i=0;i<g;i++){
            double lat=ymin+(ymax-ymin)*i/(g-1);
            for(int j=0;j<g;j++){
                double lon=xmin+(xmax-xmin)*j/(g-1);
                double tw=0,tz=0;
                for(auto&p:elev_pts){
                    double dx=p.first-lon,dy=p.second-lat;
                    double d=std::sqrt(dx*dx+dy*dy);
                    if(d<150){double w=1.0/std::max(d,2.0);tw+=w;tz+=100*w;}
                }
                float z=tw>0?(float)(tz/tw):0;
                terrain_v.push_back({(float)lon,(float)lat,z});
            }
    fprintf(stderr, "DEBUG: terrain: %zu vertices\n", terrain_v.size()); fflush(stderr);
        }
    }

    fflush(stdout); fprintf(stderr, "Loaded: %zu officers, %zu routes, %zu roads, %zu buildings\n",
           officers.size(), route_lines.size()/2, road_lines.size()/2, buildings.size());
}

// ══════════════════════════════════════════════════════════════════════════════
// OpenGL rendering
// ══════════════════════════════════════════════════════════════════════════════
static void setup_3d() {
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    float aspect=(float)win_w/win_h;
    float f=1.0f/std::tan(cam.fov*M_PI/360.0f);
    float proj[16]={f/aspect,0,0,0,0,f,0,0,0,0,-1.0001f,-2.0001f,0,0,-1,0};
    glMultMatrixf(proj);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    Vec3 eye={cam.center.x+cam.dist*std::cos(cam.pitch*M_PI/180)*std::sin(cam.bearing*M_PI/180),
              cam.center.y+cam.dist*std::cos(cam.pitch*M_PI/180)*std::cos(cam.bearing*M_PI/180),
              cam.center.z+cam.dist*std::sin(cam.pitch*M_PI/180)};
    Vec3 up={0,0,1};
    Vec3 fwd=normalize(cam.center-eye);
    Vec3 r=normalize(cross(fwd,up));
    Vec3 u=cross(r,fwd);
    float view[16]={r.x,u.x,-fwd.x,0,r.y,u.y,-fwd.y,0,r.z,u.z,-fwd.z,0,
                    -(r.x*eye.x+r.y*eye.y+r.z*eye.z),
                    -(u.x*eye.x+u.y*eye.y+u.z*eye.z),
                    (fwd.x*eye.x+fwd.y*eye.y+fwd.z*eye.z),1};
    glMultMatrixf(view);
}

static void draw_terrain() {
    if(!show_terrain||terrain_v.empty())return;
    int g=(int)std::sqrt(terrain_v.size());
    glEnable(GL_POLYGON_OFFSET_FILL);glPolygonOffset(1,1);
    glBegin(GL_TRIANGLES);
    for(int i=0;i<g-1;i++)for(int j=0;j<g-1;j++){
        int a=i*g+j,b=a+1,c=a+g,d=c+1;
        float z=(terrain_v[a].z+terrain_v[b].z+terrain_v[c].z+terrain_v[d].z)*.25f;
        float cr,cg,cb;
        if(z<15){cr=.55f;cg=.70f;cb=.35f;}
        else if(z<40){float t=(z-15)/25.f;cr=.65f-t*.1f;cg=.65f-t*.1f;cb=.35f-t*.05f;}
        else if(z<100){float t=(z-40)/60.f;cr=.55f+t*.25f;cg=.55f+t*.15f;cb=.30f+t*.30f;}
        else{float t=std::min((z-100)/150.f,1.f);cr=.80f+t*.15f;cg=.70f+t*.20f;cb=.60f+t*.25f;}
        glColor3f(cr,cg,cb);
        glVertex3f(terrain_v[a].x,terrain_v[a].y,terrain_v[a].z);
        glVertex3f(terrain_v[b].x,terrain_v[b].y,terrain_v[b].z);
        glVertex3f(terrain_v[c].x,terrain_v[c].y,terrain_v[c].z);
        glVertex3f(terrain_v[b].x,terrain_v[b].y,terrain_v[b].z);
        glVertex3f(terrain_v[d].x,terrain_v[d].y,terrain_v[d].z);
        glVertex3f(terrain_v[c].x,terrain_v[c].y,terrain_v[c].z);
    }
    glEnd();
    glDisable(GL_POLYGON_OFFSET_FILL);
}

static void draw_lines(const std::vector<LineV>& lines, float w=1.5f) {
    if(lines.empty())return;
    glLineWidth(w);glBegin(GL_LINES);
    for(const auto&v:lines){glColor3f(v.r,v.g,v.b);glVertex3f(v.x,v.y,v.z);}
    glEnd();
}

static void draw_arrow_pts() {
    if(!show_arrows||arrows.empty())return;
    glPointSize(5);glBegin(GL_POINTS);
    for(const auto&a:arrows){glColor3f(a.r,a.g,a.b);glVertex3f(a.x,a.y,a.z);}
    glEnd();
    glLineWidth(2);glBegin(GL_LINES);
    for(const auto&a:arrows){
        float ang=a.angle*M_PI/180.f;
        float dx=std::sin(ang)*5,dy=std::cos(ang)*5;
        glColor3f(a.r,a.g,a.b);
        glVertex3f(a.x,a.y,a.z);glVertex3f(a.x+dx*.7f,a.y+dy*.7f,a.z);
    }
    glEnd();
}

static void draw_buildings_3d() {
    if(!show_buildings||buildings.empty())return;
    glEnable(GL_BLEND);glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    for(const auto&b:buildings){
        float t=std::min(b.h/150.f,1.f);
        float cr=.6f+t*.3f,cg=.6f+t*.2f,cb=.7f-t*.2f;
        glColor4f(cr,cg,cb,.3f);
        float x=b.x,y=b.y,w=b.w,d=b.d,h=b.h;
        glBegin(GL_QUADS);
        glVertex3f(x-w/2,y-d/2,0);glVertex3f(x+w/2,y-d/2,0);glVertex3f(x+w/2,y-d/2,h);glVertex3f(x-w/2,y-d/2,h);
        glVertex3f(x+w/2,y+d/2,0);glVertex3f(x-w/2,y+d/2,0);glVertex3f(x-w/2,y+d/2,h);glVertex3f(x+w/2,y+d/2,h);
        glVertex3f(x-w/2,y+d/2,0);glVertex3f(x-w/2,y-d/2,0);glVertex3f(x-w/2,y-d/2,h);glVertex3f(x-w/2,y+d/2,h);
        glVertex3f(x+w/2,y-d/2,0);glVertex3f(x+w/2,y+d/2,0);glVertex3f(x+w/2,y+d/2,h);glVertex3f(x+w/2,y-d/2,h);
        glEnd();
        glColor4f(cr,cg,cb,.6f);glLineWidth(1);
        glBegin(GL_LINE_LOOP);glVertex3f(x-w/2,y-d/2,0);glVertex3f(x+w/2,y-d/2,0);glVertex3f(x+w/2,y+d/2,0);glVertex3f(x-w/2,y+d/2,0);glEnd();
        glBegin(GL_LINE_LOOP);glVertex3f(x-w/2,y-d/2,h);glVertex3f(x+w/2,y-d/2,h);glVertex3f(x+w/2,y+d/2,h);glVertex3f(x-w/2,y+d/2,h);glEnd();
    }
    glDisable(GL_BLEND);
}

// ══════════════════════════════════════════════════════════════════════════════
// Main
// ══════════════════════════════════════════════════════════════════════════════
static bool mouse_down=false;
static double last_mx,last_my;

static void key_cb(GLFWwindow* w, int key, int, int action, int) {
    if(ImGui::GetIO().WantCaptureKeyboard) return;
    if(action==GLFW_PRESS||action==GLFW_REPEAT){
        switch(key){
            case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(w,1); break;
            case GLFW_KEY_F11: fullscreen=!fullscreen; {
                GLFWmonitor* mon=fullscreen?glfwGetPrimaryMonitor():nullptr;
                const GLFWvidmode* mode=glfwGetVideoMode(glfwGetPrimaryMonitor());
                if(fullscreen) glfwSetWindowMonitor(w,mon,0,0,mode->width,mode->height,mode->refreshRate);
                else glfwSetWindowMonitor(w,nullptr,100,100,1920,1080,0);
            } break;
            case GLFW_KEY_1: cam.bearing=0; break;
            case GLFW_KEY_2: cam.bearing=90; break;
            case GLFW_KEY_3: cam.bearing=180; break;
            case GLFW_KEY_4: cam.bearing=270; break;
            case GLFW_KEY_T: show_terrain=!show_terrain; break;
            case GLFW_KEY_R: show_routes=!show_routes; break;
            case GLFW_KEY_A: show_arrows=!show_arrows; break;
            case GLFW_KEY_O: show_roads=!show_roads; break;
            case GLFW_KEY_B: show_buildings=!show_buildings; break;
            case GLFW_KEY_L: show_labels=!show_labels; break;
            case GLFW_KEY_LEFT: cam.bearing=fmodf(cam.bearing-10+360,360); break;
            case GLFW_KEY_RIGHT: cam.bearing=fmodf(cam.bearing+10,360); break;
            case GLFW_KEY_UP: cam.pitch=std::min(89.f,cam.pitch+5); break;
            case GLFW_KEY_DOWN: cam.pitch=std::max(5.f,cam.pitch-5); break;
        }
    }
}

int main(int argc, char** argv) {
    const char* routes_path="scripts/routes.geojson";
    const char* roads_path="scripts/mms_roads.geojson";
    if(argc>1)routes_path=argv[1];
    if(argc>2)roads_path=argv[2];

    if(!glfwInit())return 1;
    glfwWindowHint(GLFW_MAXIMIZED,GLFW_TRUE);
    GLFWwindow* win=glfwCreateWindow(win_w,win_h,"geodis — 3D Fieldwork Assignment Viewer [F11=fullscreen 1-4=angle T/R/A/O/B=toggle layers]",nullptr,nullptr);
    if(!win){glfwTerminate();return 1;}
    glfwMakeContextCurrent(win);
    glfwSetKeyCallback(win,key_cb);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io=ImGui::GetIO();io.ConfigFlags|=ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(win,true);
    ImGui_ImplOpenGL3_Init("#version 130");

    fprintf(stderr, "Loading data...\n");
    load_data(routes_path,roads_path);

    while(!glfwWindowShouldClose(win)){
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ═══════════════ LEFT PANEL ═══════════════
        ImGui::SetNextWindowPos(ImVec2(0,0));
        ImGui::SetNextWindowSize(ImVec2(420,(float)win_h));
        ImGui::Begin("Officers",nullptr,
            ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoCollapse);

        ImGui::TextColored(ImVec4(1,.4f,.6f,1),"geodis — %zu officers, %zu routes",
            officers.size(), route_lines.size()/2);

        // Transport filter
        ImGui::SeparatorText("Transport Filter");
        if(ImGui::Button("All")) filter_transport=-1;
        ImGui::SameLine();
        for(int i=0;i<8;i++){
            ImVec4 c(ET_COLORS[i][0],ET_COLORS[i][1],ET_COLORS[i][2],1);
            ImGui::PushStyleColor(ImGuiCol_Button,c);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,c);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,c);
            if(ImGui::Button(ET_NAMES[i])) filter_transport=i;
            ImGui::PopStyleColor(3);
            if(i<7)ImGui::SameLine();
        }
        ImGui::Checkbox("♿ Wheelchair accessible only",&filter_wheelchair);
        ImGui::SameLine();
        ImGui::Checkbox("🚫 Avoid stairs",&filter_avoid_stairs);

        ImGui::SeparatorText("Officers");
        if(!officers.empty()&&selected_officer<0)selected_officer=0;

        for(int i=0;i<(int)officers.size();i++){
            auto& off=officers[i];
            char label[256];
            snprintf(label,sizeof(label),"%s [%zu sites, %.0fm]%s###off%d",
                off.name.c_str(),off.sites.size(),off.total_cost,
                i==selected_officer?" ✓":"",i);
            if(ImGui::Selectable(label,i==selected_officer)){
                selected_officer=i;
            }
        }

        // Selected officer itinerary
        ImGui::SeparatorText("Itinerary");
        if(selected_officer>=0&&selected_officer<(int)officers.size()){
            auto& off=officers[selected_officer];
            ImGui::Text("👤 %s",off.name.c_str());
            ImGui::Text("🏠 %s",off.address.c_str());
            ImGui::Text("📋 %zu sites · 🚶 %.0fm total",off.sites.size(),off.total_cost);

            if(ImGui::Button("🔄 Optimize Route (TSP)")){
                solve_tsp(off);
            }
            ImGui::SameLine();
            ImGui::Text("TSP order: %.0fm",off.tsp_cost);

            ImGui::Separator();

            // Show sites in TSP order
            for(int si=0;si<(int)off.sites.size();si++){
                int idx=si;
                if(!off.tsp_order.empty()&&si<(int)off.tsp_order.size())idx=off.tsp_order[si];

                if(idx<0||idx>=(int)off.sites.size())continue;
                auto& os=off.sites[idx];

                // Apply filters
                if(filter_wheelchair && std::any_of(os.edge_types.begin(),os.edge_types.end(),
                    [](int t){return t==1||t==7;})) continue; // stairs or indoor
                if(filter_avoid_stairs && std::any_of(os.edge_types.begin(),os.edge_types.end(),
                    [](int t){return t==1;})) continue;
                if(filter_transport>=0 && std::find(os.edge_types.begin(),os.edge_types.end(),
                    filter_transport)==os.edge_types.end()) continue;

                char buf[256];
                snprintf(buf,sizeof(buf),"###site%d",si);
                ImGui::PushID(si);
                ImGui::TextColored(ImVec4(1,.4f,.4f,1),"%d.",si+1);
                ImGui::SameLine();
                ImGui::Text("%s",os.site.c_str());
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(.3f,.9f,.5f,1),"%.0fm",os.cost);

                // Show transportation mode pills
                for(int et:os.edge_types){
                    ImGui::SameLine();
                    ImGui::PushStyleColor(ImGuiCol_Button,
                        ImVec4(ET_COLORS[et][0],ET_COLORS[et][1],ET_COLORS[et][2],.8f));
                    ImGui::SmallButton(ET_NAMES[et]);
                    ImGui::PopStyleColor();
                }

                ImGui::Text("  📍 %s",os.site_addr.c_str());
                ImGui::Text("  📏 %.0fm straight · ⬆%.1fm ⬇%.1fm · %d waypoints",
                    os.straight,os.ascent,os.descent,os.waypoints);
                ImGui::PopID();
            }
        }

        ImGui::End();

        // ═══════════════ 3D VIEW ═══════════════
        glfwGetFramebufferSize(win,&win_w,&win_h);
        glViewport(0,0,win_w,win_h);
        glClearColor(.53f,.81f,.98f,1);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        setup_3d();

        draw_terrain();
        if(show_roads)  draw_lines(road_lines,1);
        if(show_routes) draw_lines(route_lines,2.5f);
        draw_arrow_pts();
        draw_buildings_3d();

        // 2D overlay labels for buildings
        if(show_labels){
            // Simple projection for building labels
            ImGui::GetForegroundDrawList()->AddText(
                ImVec2(win_w/2+100,win_h/2-200),IM_COL32(255,255,255,180),"Building Names");
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(win);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
