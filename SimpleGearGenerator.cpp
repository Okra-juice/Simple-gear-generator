// ============================================================================
//  SimpleGearGenerator.cpp  ——  Simple Gear Generator
//  功能：三种齿轮类型（普通齿轮 / 冠状轮(桶状) / 垂直轮(冠齿轮)），
//        中英双语界面，右侧实时 3D 预览（拖拽旋转 + 滚轮缩放），
//        导出 STL / STEP，单位可选 mm/cm/inch/m，
//        底部 RichEdit 日志窗口（黑色终端风格，英文输出，错误标红）。
//
//  编译方式（任选其一）：
//    MSVC :  cl /EHsc /O2 /utf-8 SimpleGearGenerator.cpp /link /SUBSYSTEM:WINDOWS
//    MinGW:  g++ -O2 SimpleGearGenerator.cpp -o SimpleGearGenerator.exe -mwindows -lcomctl32 -lcomdlg32
// ============================================================================

#ifndef UNICODE
#define UNICODE
#endif
#ifndef _UNICODE
#define _UNICODE
#endif

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commctrl.h>
#include <commdlg.h>
#include <richedit.h>   // RichEdit 控件（彩色日志）

#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cwchar>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cstdarg>

#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "comdlg32.lib")

#if defined(_MSC_VER)
#pragma comment(linker, "\"/manifestdependency:type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif

using namespace std;

const double PI = 3.14159265358979323846;

// ---------------- 几何数据结构 ----------------
struct Vec2 { double x, y; };
struct Vec3 { double x, y, z; };
struct Triangle { Vec3 v[3]; Vec3 n; };
struct Mesh { vector<Triangle> tris; };

// 中心孔形状
enum HoleShape { HS_CIRCLE, HS_D, HS_SQUARE, HS_POLYGON, HS_STAR };
// 倒角位置
enum ChamferLoc { CL_NONE, CL_TOP, CL_BOTTOM, CL_BOTH, CL_TIP };
// 齿轮类型
enum GearType { GT_SPUR, GT_CROWNED, GT_CROWNWHEEL, GT_INTERNAL, GT_STACKED };

struct GearParams {
    double outerDiameter;   // 外径（齿顶圆直径 / 冠齿轮外径）
    int    teeth;           // 齿数
    double thickness;       // 厚度（普通 / 冠状轮）
    bool   hasBoss;         // 是否有中心凸起
    double bossThickness;   // 凸起厚度
    double bossDiameter;    // 凸起直径
    bool   hasHole;         // 是否有中心孔
    HoleShape holeShape;    // 孔形状（默认圆形）
    double holeDiameter;    // 孔直径（基准尺寸）
    double dLength;         // D 型孔长
    double dWidth;          // D 型孔宽
    int    holeSides;       // 多边形边数 / 星形角数
    double filletRadius;    // 冠状轮：端面圆角半径
    double sheetThickness;  // 冠齿轮：薄片厚度
    double toothHeight;     // 冠齿轮：齿高
    bool   hasChamfer;      // 是否启用倒角
    double chamferSize;     // 倒角大小
    int    chamferLoc;      // 倒角位置（ChamferLoc 枚举值）
};

// 多层齿轮：单层信息（齿轮类型 + 完整参数）
struct LayerInfo {
    GearType   type;
    GearParams params;
};

struct GearGeom {
    double m, r, ra, rf, rb;
    double psiP, invAlpha;
    double thetaA, thetaB;
};

// ---------------- 基础数学 ----------------
static Vec3 sub3(const Vec3& a, const Vec3& b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
static Vec3 cross3(const Vec3& a, const Vec3& b) {
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
static Vec2 rot2(const Vec2& p, double a) {
    double c = cos(a), s = sin(a);
    return { p.x * c - p.y * s, p.x * s + p.y * c };
}
static double safeAcos(double x) {
    if (x > 1.0) x = 1.0;
    if (x < -1.0) x = -1.0;
    return acos(x);
}
static double vlen2(double x, double y) { return sqrt(x * x + y * y); }

// 求解 inv(a) = tan(a) - a = C，返回 a
static double solveInv(double C) {
    if (C <= 0.0) return 0.0;
    double lo = 0.0, hi = 1.45;
    for (int i = 0; i < 80; ++i) {
        double mid = (lo + hi) * 0.5;
        if (tan(mid) - mid < C) lo = mid; else hi = mid;
    }
    return (lo + hi) * 0.5;
}

// ---------------- 网格构建 ----------------
static void addTri(Mesh& m, const Vec3& a, const Vec3& b, const Vec3& c) {
    Triangle t;
    t.v[0] = a; t.v[1] = b; t.v[2] = c;
    Vec3 n = cross3(sub3(b, a), sub3(c, a));
    double l = sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    if (l > 1e-12) { n.x /= l; n.y /= l; n.z /= l; } else { n = { 0, 0, 1 }; }
    t.n = n;
    m.tris.push_back(t);
}

static GearGeom computeGeom(const GearParams& p) {
    GearGeom g;
    g.m  = p.outerDiameter / (p.teeth + 2.0);
    g.r  = g.m * p.teeth / 2.0;
    g.ra = p.outerDiameter / 2.0;
    g.rf = g.r - 1.25 * g.m;
    double alpha = 20.0 * PI / 180.0;
    g.rb = g.r * cos(alpha);
    g.invAlpha = tan(alpha) - alpha;
    g.psiP = PI / (2.0 * p.teeth);

    double rmin = max(g.rb, g.rf);
    double alphaA = safeAcos(g.rb / g.ra);
    double invA = tan(alphaA) - alphaA;
    g.thetaA = g.psiP + g.invAlpha - invA;
    double alphaM = safeAcos(g.rb / rmin);
    double invM = tan(alphaM) - alphaM;
    g.thetaB = g.psiP + g.invAlpha - invM;

    if (g.thetaA < 0.0) g.thetaA = 0.0;
    if (g.thetaB <= g.thetaA + 1e-9) g.thetaB = g.thetaA + 1e-9;
    return g;
}

static vector<Vec2> buildHalfProfile(const GearGeom& g, int teeth) {
    vector<Vec2> pts;
    const int nLand = 4, nFlank = 12, nRoot = 3;

    for (int i = 0; i <= nLand; ++i) {
        double th = g.thetaA * i / nLand;
        pts.push_back({ g.ra * cos(th), g.ra * sin(th) });
    }
    for (int i = 1; i <= nFlank; ++i) {
        double th = g.thetaA + (g.thetaB - g.thetaA) * i / nFlank;
        double C = g.psiP + g.invAlpha - th;
        double aRho = solveInv(C);
        double rho = g.rb / cos(aRho);
        pts.push_back({ rho * cos(th), rho * sin(th) });
    }
    double thetaF = PI / teeth;
    Vec2 Pf = pts.back();
    Vec2 Q  = { g.rf * cos(thetaF), g.rf * sin(thetaF) };
    for (int i = 1; i <= nRoot; ++i) {
        double t = (double)i / nRoot;
        pts.push_back({ Pf.x + (Q.x - Pf.x) * t, Pf.y + (Q.y - Pf.y) * t });
    }
    return pts;
}

static vector<Vec2> buildGearContour(const GearParams& p, const GearGeom& g) {
    vector<Vec2> half = buildHalfProfile(g, p.teeth);
    int N = (int)half.size();
    vector<Vec2> contour;
    contour.reserve((size_t)p.teeth * (N + N - 2));
    double pitch = 2.0 * PI / p.teeth;
    for (int k = 0; k < p.teeth; ++k) {
        double base = k * pitch;
        for (int i = 0; i < N; ++i)
            contour.push_back(rot2(half[i], base));
        for (int i = N - 2; i >= 1; --i) {
            Vec2 mir = { half[i].x, -half[i].y };
            contour.push_back(rot2(mir, base + pitch));
        }
    }
    return contour;
}

static vector<Vec2> makeCircle(double radius, int segments) {
    vector<Vec2> c;
    c.reserve(segments);
    for (int i = 0; i < segments; ++i) {
        double a = 2.0 * PI * i / segments;
        c.push_back({ radius * cos(a), radius * sin(a) });
    }
    return c;
}

// 将轮廓沿法向向内偏移 r（顶点法向取相邻边法向的平均，CCW 轮廓）
static vector<Vec2> offsetContourInward(const vector<Vec2>& c, double r) {
    int n = (int)c.size();
    vector<Vec2> out(n);
    for (int i = 0; i < n; ++i) {
        const Vec2& a = c[(i - 1 + n) % n];
        const Vec2& b = c[i];
        const Vec2& d = c[(i + 1) % n];
        double tx = b.x - a.x, ty = b.y - a.y;
        double ux = d.x - b.x, uy = d.y - b.y;
        double n1x = -ty, n1y = tx;   // CCW 轮廓向内法向
        double n2x = -uy, n2y = ux;
        double l1 = vlen2(n1x, n1y), l2 = vlen2(n2x, n2y);
        if (l1 < 1e-12) { n1x = n2x; n1y = n2y; l1 = l2; }
        if (l2 < 1e-12) { n2x = n1x; n2y = n1y; l2 = l1; }
        n1x /= l1; n1y /= l1;
        n2x /= l2; n2y /= l2;
        double nx = n1x + n2x, ny = n1y + n2y;
        double l = vlen2(nx, ny);
        if (l < 1e-12) { out[i] = b; continue; }
        nx /= l; ny /= l;
        out[i] = { b.x + nx * r, b.y + ny * r };
    }
    return out;
}

// 按比例缩放轮廓（关于原点）
static vector<Vec2> scaleContour(const vector<Vec2>& c, double s) {
    vector<Vec2> out(c.size());
    for (size_t i = 0; i < c.size(); ++i) out[i] = { c[i].x * s, c[i].y * s };
    return out;
}

static vector<Vec2> circleAtAngles(double radius, const vector<Vec2>& outer) {
    vector<Vec2> c;
    c.reserve(outer.size());
    double prev = 0.0;
    for (size_t i = 0; i < outer.size(); ++i) {
        double a = atan2(outer[i].y, outer[i].x);
        while (a < prev - PI) a += 2.0 * PI;
        while (a > prev + PI) a -= 2.0 * PI;
        prev = a;
        c.push_back({ radius * cos(a), radius * sin(a) });
    }
    return c;
}

// ---------------- 孔形状轮廓 ----------------
static vector<Vec2> makeSquare(double side) {
    double h = side / 2.0;
    return { { h, -h }, { h, h }, { -h, h }, { -h, -h } };
}
static vector<Vec2> makePolygon(double circumRadius, int sides) {
    vector<Vec2> p;
    p.reserve(sides);
    for (int i = 0; i < sides; ++i) {
        double a = 2.0 * PI * i / sides;
        p.push_back({ circumRadius * cos(a), circumRadius * sin(a) });
    }
    return p;
}
static vector<Vec2> makeStar(double outerR, int points) {
    double innerR = outerR * 0.5;
    vector<Vec2> p;
    p.reserve(points * 2);
    for (int i = 0; i < points; ++i) {
        double a0 = 2.0 * PI * i / points;
        double a1 = a0 + PI / points;
        p.push_back({ outerR * cos(a0), outerR * sin(a0) });
        p.push_back({ innerR * cos(a1), innerR * sin(a1) });
    }
    return p;
}
static vector<Vec2> makeDShape(double L, double W) {
    double R = L / 2.0;
    double flatX = R - W;
    double c = sqrt(max(0.0, R * R - flatX * flatX));
    vector<Vec2> pts;
    int nFlat = 6, nArc = 48;
    for (int i = 0; i <= nFlat; ++i) {
        double y = c - 2.0 * c * i / nFlat;
        pts.push_back({ flatX, y });
    }
    double aTop = atan2(c, flatX);
    for (int i = 1; i <= nArc; ++i) {
        double a = -aTop + 2.0 * aTop * i / nArc;
        pts.push_back({ R * cos(a), R * sin(a) });
    }
    return pts;
}
static vector<Vec2> buildHolePolygon(const GearParams& p) {
    switch (p.holeShape) {
    case HS_CIRCLE:  return makeCircle(p.holeDiameter * 0.5, 128);
    case HS_SQUARE:  return makeSquare(p.holeDiameter);
    case HS_D:       return makeDShape(p.dLength, p.dWidth);
    case HS_POLYGON: return makePolygon(p.holeDiameter * 0.5, p.holeSides);
    default:         return makeStar(p.holeDiameter * 0.5, p.holeSides);
    }
}
static double polygonRadiusAt(const vector<Vec2>& poly, double angle) {
    Vec2 dir = { cos(angle), sin(angle) };
    double best = 1e18;
    int n = (int)poly.size();
    for (int i = 0; i < n; ++i) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[(i + 1) % n];
        double abx = b.x - a.x, aby = b.y - a.y;
        double denom = dir.x * aby - dir.y * abx;
        if (fabs(denom) < 1e-12) continue;
        double t = (a.x * aby - a.y * abx) / denom;
        double s = (a.x * dir.y - a.y * dir.x) / denom;
        if (t > 0 && s >= -1e-9 && s <= 1.0 + 1e-9) {
            if (t < best) best = t;
        }
    }
    return best;
}
static vector<Vec2> holeAtAngles(const vector<Vec2>& holePoly, const vector<Vec2>& outer) {
    vector<Vec2> c;
    c.reserve(outer.size());
    double prev = 0.0;
    for (size_t i = 0; i < outer.size(); ++i) {
        double a = atan2(outer[i].y, outer[i].x);
        while (a < prev - PI) a += 2.0 * PI;
        while (a > prev + PI) a -= 2.0 * PI;
        prev = a;
        double r = polygonRadiusAt(holePoly, a);
        c.push_back({ r * cos(a), r * sin(a) });
    }
    return c;
}

static void addAnnulusTop(Mesh& m, const vector<Vec2>& outer, const vector<Vec2>& inner, double z) {
    int n = (int)outer.size();
    vector<Vec2> mid(n);
    for (int i = 0; i < n; ++i) mid[i] = { (outer[i].x + inner[i].x) * 0.5, (outer[i].y + inner[i].y) * 0.5 };
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        addTri(m, { outer[i].x, outer[i].y, z }, { outer[j].x, outer[j].y, z }, { mid[j].x, mid[j].y, z });
        addTri(m, { outer[i].x, outer[i].y, z }, { mid[j].x, mid[j].y, z }, { mid[i].x, mid[i].y, z });
        addTri(m, { mid[i].x, mid[i].y, z }, { mid[j].x, mid[j].y, z }, { inner[j].x, inner[j].y, z });
        addTri(m, { mid[i].x, mid[i].y, z }, { inner[j].x, inner[j].y, z }, { inner[i].x, inner[i].y, z });
    }
}
static void addAnnulusBottom(Mesh& m, const vector<Vec2>& outer, const vector<Vec2>& inner, double z) {
    int n = (int)outer.size();
    vector<Vec2> mid(n);
    for (int i = 0; i < n; ++i) mid[i] = { (outer[i].x + inner[i].x) * 0.5, (outer[i].y + inner[i].y) * 0.5 };
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        addTri(m, { outer[i].x, outer[i].y, z }, { mid[j].x, mid[j].y, z }, { outer[j].x, outer[j].y, z });
        addTri(m, { outer[i].x, outer[i].y, z }, { mid[i].x, mid[i].y, z }, { mid[j].x, mid[j].y, z });
        addTri(m, { mid[i].x, mid[i].y, z }, { inner[j].x, inner[j].y, z }, { mid[j].x, mid[j].y, z });
        addTri(m, { mid[i].x, mid[i].y, z }, { inner[i].x, inner[i].y, z }, { inner[j].x, inner[j].y, z });
    }
}
static void addDiscTop(Mesh& m, const vector<Vec2>& ring, double z) {
    int n = (int)ring.size();
    vector<Vec2> mid(n);
    for (int i = 0; i < n; ++i) mid[i] = { ring[i].x * 0.5, ring[i].y * 0.5 };
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        addTri(m, { 0, 0, z }, { mid[i].x, mid[i].y, z }, { mid[j].x, mid[j].y, z });
        addTri(m, { mid[i].x, mid[i].y, z }, { ring[i].x, ring[i].y, z }, { ring[j].x, ring[j].y, z });
        addTri(m, { mid[i].x, mid[i].y, z }, { ring[j].x, ring[j].y, z }, { mid[j].x, mid[j].y, z });
    }
}
static void addDiscBottom(Mesh& m, const vector<Vec2>& ring, double z) {
    int n = (int)ring.size();
    vector<Vec2> mid(n);
    for (int i = 0; i < n; ++i) mid[i] = { ring[i].x * 0.5, ring[i].y * 0.5 };
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        addTri(m, { 0, 0, z }, { mid[j].x, mid[j].y, z }, { mid[i].x, mid[i].y, z });
        addTri(m, { ring[i].x, ring[i].y, z }, { mid[j].x, mid[j].y, z }, { ring[j].x, ring[j].y, z });
        addTri(m, { ring[i].x, ring[i].y, z }, { mid[i].x, mid[i].y, z }, { mid[j].x, mid[j].y, z });
    }
}
static void addOuterWall(Mesh& m, const vector<Vec2>& pts, double z0, double z1) {
    int n = (int)pts.size();
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        addTri(m, { pts[i].x, pts[i].y, z0 }, { pts[j].x, pts[j].y, z0 }, { pts[j].x, pts[j].y, z1 });
        addTri(m, { pts[i].x, pts[i].y, z0 }, { pts[j].x, pts[j].y, z1 }, { pts[i].x, pts[i].y, z1 });
    }
}
static void addInnerWall(Mesh& m, const vector<Vec2>& pts, double z0, double z1) {
    int n = (int)pts.size();
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        addTri(m, { pts[j].x, pts[j].y, z0 }, { pts[i].x, pts[i].y, z0 }, { pts[i].x, pts[i].y, z1 });
        addTri(m, { pts[j].x, pts[j].y, z0 }, { pts[i].x, pts[i].y, z1 }, { pts[j].x, pts[j].y, z1 });
    }
}
// 两个轮廓（点数与索引一一对应）之间的竖直放样侧壁，a 为下方(z0)，b 为上方(z1)
static void addWallBetween(Mesh& m, const vector<Vec2>& a, double za, const vector<Vec2>& b, double zb) {
    int n = (int)min(a.size(), b.size());
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        addTri(m, { a[i].x, a[i].y, za }, { a[j].x, a[j].y, za }, { b[j].x, b[j].y, zb });
        addTri(m, { a[i].x, a[i].y, za }, { b[j].x, b[j].y, zb }, { b[i].x, b[i].y, zb });
    }
}
// 凸多边形端盖（从质心三角化，三角形更小更均匀），top 为 +z 法向，否则 -z
static void addConvexCap(Mesh& m, const vector<Vec2>& poly, double z, bool top) {
    int n = (int)poly.size();
    Vec2 c = { 0, 0 };
    for (int i = 0; i < n; ++i) { c.x += poly[i].x; c.y += poly[i].y; }
    c.x /= n; c.y /= n;
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        if (top)
            addTri(m, { c.x, c.y, z }, { poly[i].x, poly[i].y, z }, { poly[j].x, poly[j].y, z });
        else
            addTri(m, { c.x, c.y, z }, { poly[j].x, poly[j].y, z }, { poly[i].x, poly[i].y, z });
    }
}
// 环扇区轮廓（CCW）：外弧 -w..+w，内弧 +w..-w
static vector<Vec2> makeSector(double Ro, double Ri, double w) {
    int nO = 6, nI = 6;
    vector<Vec2> pts;
    pts.reserve(nO + nI);
    for (int i = 0; i < nO; ++i) {
        double a = -w + (2.0 * w * i) / (nO - 1);
        pts.push_back({ Ro * cos(a), Ro * sin(a) });
    }
    for (int i = nI - 1; i >= 0; --i) {
        double a = -w + (2.0 * w * i) / (nI - 1);
        pts.push_back({ Ri * cos(a), Ri * sin(a) });
    }
    return pts;
}

// ---------------- 齿轮网格 ----------------
// 普通齿轮（支持可选倒角）
static Mesh buildGearMesh(const GearParams& p) {
    Mesh m;
    GearGeom g = computeGeom(p);
    vector<Vec2> outer = buildGearContour(p, g);

    double T  = p.thickness;
    double rb = p.hasBoss ? p.bossDiameter * 0.5 : 0.0;
    double tb = p.hasBoss ? p.bossThickness : 0.0;
    double zTop = T + tb;

    // 倒角设置
    bool doChamfer = p.hasChamfer && p.chamferSize > 0;
    bool chamfBottom = doChamfer && (p.chamferLoc == CL_BOTTOM || p.chamferLoc == CL_BOTH);
    bool chamfTop    = doChamfer && (p.chamferLoc == CL_TOP || p.chamferLoc == CL_BOTH);
    bool chamfTip    = doChamfer && (p.chamferLoc == CL_TIP);
    double chamfH = doChamfer ? min(p.chamferSize, T * 0.4) : 0.0;

    // 齿尖倒角：缩小齿顶圆轮廓
    if (chamfTip) {
        double s = max(0.01, (g.ra - p.chamferSize) / g.ra);
        outer = scaleContour(outer, s);
    }

    // 顶/底边倒角用的缩小轮廓
    vector<Vec2> outerChamf;
    if (chamfBottom || chamfTop) {
        double s = max(0.01, (g.ra - p.chamferSize) / g.ra);
        outerChamf = scaleContour(outer, s);
    }

    vector<Vec2> holePoly;
    if (p.hasHole) holePoly = buildHolePolygon(p);

    // 底面
    if (chamfBottom) {
        if (p.hasHole) addAnnulusBottom(m, outerChamf, holeAtAngles(holePoly, outerChamf), 0.0);
        else           addDiscBottom(m, outerChamf, 0.0);
        addWallBetween(m, outerChamf, 0.0, outer, chamfH);
    } else {
        if (p.hasHole) addAnnulusBottom(m, outer, holeAtAngles(holePoly, outer), 0.0);
        else           addDiscBottom(m, outer, 0.0);
    }

    // 外壁
    double wallStart = chamfBottom ? chamfH : 0.0;
    double wallEnd   = chamfTop ? T - chamfH : T;
    addOuterWall(m, outer, wallStart, wallEnd);

    // 顶面
    if (chamfTop) {
        addWallBetween(m, outer, T - chamfH, outerChamf, T);
        if (p.hasBoss)      addAnnulusTop(m, outerChamf, circleAtAngles(rb, outerChamf), T);
        else if (p.hasHole) addAnnulusTop(m, outerChamf, holeAtAngles(holePoly, outerChamf), T);
        else                addDiscTop(m, outerChamf, T);
    } else {
        if (p.hasBoss)      addAnnulusTop(m, outer, circleAtAngles(rb, outer), T);
        else if (p.hasHole) addAnnulusTop(m, outer, holeAtAngles(holePoly, outer), T);
        else                addDiscTop(m, outer, T);
    }

    if (p.hasBoss) {
        vector<Vec2> bossCircle = makeCircle(rb, 128);
        addOuterWall(m, bossCircle, T, T + tb);
        if (p.hasHole) addAnnulusTop(m, bossCircle, holeAtAngles(holePoly, bossCircle), T + tb);
        else           addDiscTop(m, bossCircle, T + tb);
    }
    if (p.hasHole) addInnerWall(m, holePoly, 0.0, zTop);
    return m;
}

// 冠状轮（桶状/鼓形）：齿沿轴向呈鼓形分布，中间粗两端细
static Mesh buildCrownedGearMesh(const GearParams& p) {
    Mesh m;
    GearGeom g = computeGeom(p);
    vector<Vec2> outer = buildGearContour(p, g);

    double T = p.thickness;
    double barrel = p.filletRadius;  // 桶形量（中间径向膨胀量）
    double ra = g.ra;                // 齿顶圆半径
    double rb = p.hasBoss ? p.bossDiameter * 0.5 : 0.0;
    double tb = p.hasBoss ? p.bossThickness : 0.0;

    vector<Vec2> holePoly;
    if (p.hasHole) holePoly = buildHolePolygon(p);

    const int nSeg = 20;

    // 桶形缩放因子：两端=1.0，中间=1.0+barrel/ra
    auto scaleAt = [&](double z) -> double {
        return 1.0 + (barrel / ra) * sin(PI * z / T);
    };

    // 底面 (z=0, scale=1.0)
    if (p.hasHole) addAnnulusBottom(m, outer, holeAtAngles(holePoly, outer), 0.0);
    else           addDiscBottom(m, outer, 0.0);

    // 桶形侧壁（沿轴向分层放样）
    {
        vector<Vec2> prev = outer;
        double prevZ = 0.0;
        for (int k = 1; k <= nSeg; ++k) {
            double z = T * (double)k / nSeg;
            vector<Vec2> cur = scaleContour(outer, scaleAt(z));
            addWallBetween(m, prev, prevZ, cur, z);
            prev = cur; prevZ = z;
        }
    }

    // 顶面 (z=T, scale=1.0)
    if (p.hasBoss)      addAnnulusTop(m, outer, circleAtAngles(rb, outer), T);
    else if (p.hasHole) addAnnulusTop(m, outer, holeAtAngles(holePoly, outer), T);
    else                addDiscTop(m, outer, T);

    // 中心凸起（直筒，在顶面 z=T 之上）
    if (p.hasBoss) {
        vector<Vec2> bc = makeCircle(rb, 128);
        addOuterWall(m, bc, T, T + tb);
        if (p.hasHole) addAnnulusTop(m, bc, holeAtAngles(holePoly, bc), T + tb);
        else           addDiscTop(m, bc, T + tb);
    }

    // 中心孔（直筒，贯穿齿轮与凸起）
    if (p.hasHole) {
        double zTop = T + (p.hasBoss ? tb : 0.0);
        addInnerWall(m, holePoly, 0.0, zTop);
    }
    return m;
}

// 单个冠齿轮梯形齿（绕 z 旋转 th）
static void addCrownTooth(Mesh& m, double Ro, double Ri, double Ro1, double Ri1,
                          double w0, double w1, double z0, double h, double th) {
    vector<Vec2> base = makeSector(Ro, Ri, w0);
    vector<Vec2> top  = makeSector(Ro1, Ri1, w1);
    for (size_t i = 0; i < base.size(); ++i) base[i] = rot2(base[i], th);
    for (size_t i = 0; i < top.size();  ++i) top[i]  = rot2(top[i], th);
    addWallBetween(m, base, z0, top, z0 + h);
    addConvexCap(m, top, z0 + h, true);
}

// 垂直轮（皇冠冠齿轮）：圆盘薄片 + 一圈梯形齿 + 可选中心凸起
static Mesh buildCrownWheelMesh(const GearParams& p) {
    Mesh m;
    double R = p.outerDiameter * 0.5;
    double t = p.sheetThickness;
    double h = p.toothHeight;
    int N = p.teeth;

    double rb = p.hasBoss ? p.bossDiameter * 0.5 : 0.0;
    double tb = p.hasBoss ? p.bossThickness : 0.0;

    vector<Vec2> disc = makeCircle(R, 128);
    vector<Vec2> holePoly;
    if (p.hasHole) holePoly = buildHolePolygon(p);

    // 底面
    if (p.hasHole) addAnnulusBottom(m, disc, holeAtAngles(holePoly, disc), 0.0);
    else           addDiscBottom(m, disc, 0.0);

    // 顶面（有凸起时留出凸起位置）
    if (p.hasBoss)      addAnnulusTop(m, disc, circleAtAngles(rb, disc), t);
    else if (p.hasHole) addAnnulusTop(m, disc, holeAtAngles(holePoly, disc), t);
    else                addDiscTop(m, disc, t);

    addOuterWall(m, disc, 0.0, t);

    double Ri = R * 0.62;
    double pitch = 2.0 * PI / N;
    double w0 = pitch * 0.26;
    double w1 = w0 * 0.5;
    double Ri1 = Ri + (R - Ri) * 0.18;

    for (int k = 0; k < N; ++k) {
        double th = k * pitch;
        addCrownTooth(m, R, Ri, R, Ri1, w0, w1, t, h, th);
    }

    // 填充中心区域（从薄片顶面到齿高），使齿之间不再空心
    {
        vector<Vec2> centerRing = makeCircle(Ri, 128);
        addOuterWall(m, centerRing, t, t + h);
        if (p.hasBoss && rb > 0 && rb < Ri)
            addAnnulusTop(m, centerRing, circleAtAngles(rb, centerRing), t + h);
        else if (p.hasHole)
            addAnnulusTop(m, centerRing, holeAtAngles(holePoly, centerRing), t + h);
        else
            addDiscTop(m, centerRing, t + h);
    }

    // 中心凸起（hub）：在中心填充柱顶部（z=t+h）之上再向上凸起 tb
    if (p.hasBoss) {
        vector<Vec2> bc = makeCircle(rb, 128);
        addOuterWall(m, bc, t + h, t + h + tb);
        if (p.hasHole) addAnnulusTop(m, bc, holeAtAngles(holePoly, bc), t + h + tb);
        else           addDiscTop(m, bc, t + h + tb);
    }

    // 中心孔（贯穿薄片、中心填充柱与凸起）
    if (p.hasHole) {
        double zTop = t + h + (p.hasBoss ? tb : 0.0);
        addInnerWall(m, holePoly, 0.0, zTop);
    }
    return m;
}

// 将轮廓各点关于节圆反射：rho → 2*r - rho（角度不变），用于内齿轮齿廓
static vector<Vec2> reflectThroughPitch(const vector<Vec2>& contour, double r) {
    vector<Vec2> out(contour.size());
    for (size_t i = 0; i < contour.size(); ++i) {
        double rho = sqrt(contour[i].x * contour[i].x + contour[i].y * contour[i].y);
        if (rho < 1e-12) { out[i] = contour[i]; continue; }
        double factor = (2.0 * r - rho) / rho;
        out[i] = { contour[i].x * factor, contour[i].y * factor };
    }
    return out;
}

// 内齿轮：圆环外圈光滑，内圈带齿（齿向内凸起），外径为圆环外径
static Mesh buildInternalGearMesh(const GearParams& p) {
    Mesh m;
    // 用等效外径计算外齿轮几何，再关于节圆反射得到内齿轮齿廓
    double module = p.outerDiameter / (p.teeth + 6.5);
    double effectiveOD = module * (p.teeth + 2.0);
    GearParams extP = p;
    extP.outerDiameter = effectiveOD;
    extP.hasChamfer = false; // 内齿轮倒角单独处理
    GearGeom g = computeGeom(extP);
    vector<Vec2> extContour = buildGearContour(extP, g);
    double r = g.r;
    vector<Vec2> intContour = reflectThroughPitch(extContour, r);
    double ringR = p.outerDiameter * 0.5;
    double T = p.thickness;

    vector<Vec2> outerRing = circleAtAngles(ringR, intContour);

    // 倒角
    bool doChamfer = p.hasChamfer && p.chamferSize > 0;
    bool chamfBottom = doChamfer && (p.chamferLoc == CL_BOTTOM || p.chamferLoc == CL_BOTH);
    bool chamfTop = doChamfer && (p.chamferLoc == CL_TOP || p.chamferLoc == CL_BOTH);
    double chamfH = doChamfer ? min(p.chamferSize, T * 0.4) : 0.0;

    vector<Vec2> ringChamf, intChamf;
    if (chamfBottom || chamfTop) {
        double ringS = max(0.01, (ringR - p.chamferSize) / ringR);
        ringChamf = scaleContour(outerRing, ringS);
        // 内齿倒角：将内齿轮廓各点向内（向圆心方向）偏移
        double intS = max(0.01, 1.0 - p.chamferSize / ringR);
        intChamf = scaleContour(intContour, intS);
    }

    // 底面
    if (chamfBottom) {
        addAnnulusBottom(m, ringChamf, intChamf, 0.0);
        addWallBetween(m, ringChamf, 0.0, outerRing, chamfH);
        addWallBetween(m, intContour, chamfH, intChamf, 0.0);
    } else {
        addAnnulusBottom(m, outerRing, intContour, 0.0);
    }

    // 外壁（光滑圆）+ 内壁（齿廓）
    double wallStart = chamfBottom ? chamfH : 0.0;
    double wallEnd = chamfTop ? T - chamfH : T;
    addOuterWall(m, outerRing, wallStart, wallEnd);
    addInnerWall(m, intContour, wallStart, wallEnd);

    // 顶面
    if (chamfTop) {
        addWallBetween(m, outerRing, T - chamfH, ringChamf, T);
        addWallBetween(m, intChamf, T, intContour, T - chamfH);
        addAnnulusTop(m, ringChamf, intChamf, T);
    } else {
        addAnnulusTop(m, outerRing, intContour, T);
    }
    return m;
}

static Mesh buildMesh(const GearParams& p, GearType t) {
    switch (t) {
    case GT_CROWNED:    return buildCrownedGearMesh(p);
    case GT_INTERNAL:   return buildInternalGearMesh(p);
    case GT_CROWNWHEEL: return buildCrownWheelMesh(p);
    case GT_STACKED:    return Mesh(); // 多层齿轮由 buildStackedMesh 处理
    default:            return buildGearMesh(p);
    }
}

// 多层齿轮：计算单层总高度（齿轮体 + 凸起）
static double layerHeight(const LayerInfo& li) {
    double h = (li.type == GT_CROWNWHEEL)
        ? (li.params.sheetThickness + li.params.toothHeight)
        : li.params.thickness;
    if (li.params.hasBoss) h += li.params.bossThickness;
    return h;
}

// 多层齿轮：将多个不同类型齿轮沿 Z 轴依次堆叠
static Mesh buildStackedMesh(const vector<LayerInfo>& layers) {
    Mesh result;
    double zOffset = 0.0;
    for (size_t i = 0; i < layers.size(); ++i) {
        const GearParams& p = layers[i].params;
        GearType gt = layers[i].type;
        Mesh layerMesh = buildMesh(p, gt);
        for (auto& t : layerMesh.tris) {
            for (int k = 0; k < 3; ++k)
                t.v[k].z += zOffset;
        }
        result.tris.insert(result.tris.end(), layerMesh.tris.begin(), layerMesh.tris.end());
        zOffset += layerHeight(layers[i]);
    }
    return result;
}

// 写二进制 STL
static bool writeBinarySTL(const wchar_t* path, const Mesh& m, double scale) {
    FILE* f = _wfopen(path, L"wb");
    if (!f) return false;

    char header[80];
    memset(header, 0, 80);
    memcpy(header, "Simple Gear Generator STL", 26);
    fwrite(header, 1, 80, f);

    uint32_t n = (uint32_t)m.tris.size();
    fwrite(&n, 4, 1, f);

    for (size_t i = 0; i < m.tris.size(); ++i) {
        const Triangle& t = m.tris[i];
        float nx = (float)t.n.x, ny = (float)t.n.y, nz = (float)t.n.z;
        fwrite(&nx, 4, 1, f); fwrite(&ny, 4, 1, f); fwrite(&nz, 4, 1, f);
        for (int k = 0; k < 3; ++k) {
            float x = (float)(t.v[k].x * scale), y = (float)(t.v[k].y * scale), z = (float)(t.v[k].z * scale);
            fwrite(&x, 4, 1, f); fwrite(&y, 4, 1, f); fwrite(&z, 4, 1, f);
        }
        uint16_t attr = 0;
        fwrite(&attr, 2, 1, f);
    }
    fclose(f);
    return true;
}

// 写 STEP (AP214) 文件——每个三角形作为一个 Advanced_Face
static bool writeSTEP(const wchar_t* path, const Mesh& m, double scale) {
    FILE* f = _wfopen(path, L"wb");
    if (!f) return false;

    fprintf(f, "ISO-10303-21;\n");
    fprintf(f, "HEADER;\n");
    fprintf(f, "FILE_DESCRIPTION(('Simple Gear Generator STEP output'),'2;1');\n");
    fprintf(f, "FILE_NAME('gear.step','2026-08-17T00:00:00',('User'),(''),'Simple-Gear-Generator 1.0','Trae','');\n");
    fprintf(f, "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n");
    fprintf(f, "ENDSEC;\n");
    fprintf(f, "DATA;\n");

    int id = 1;

    // 单位与几何上下文
    int idLenUnit = id++;
    fprintf(f, "#%d=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n", idLenUnit);
    int idAngUnit = id++;
    fprintf(f, "#%d=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));\n", idAngUnit);
    int idSolidAngUnit = id++;
    fprintf(f, "#%d=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.));\n", idSolidAngUnit);
    int idUncert = id++;
    fprintf(f, "#%d=(CONVERSION_BASED_UNIT('DEGREE',#%d)NAMED_UNIT(#%d)PLANE_ANGLE_UNIT());\n", idUncert, idUncert + 1, idUncert + 2);
    int idDegConv = id++;
    fprintf(f, "#%d=PLANE_ANGLE_MEASURE_WITH_UNIT(1.74532925199433E-2,#%d);\n", idDegConv, idAngUnit);
    int idDegCtx = id++;
    fprintf(f, "#%d=DIMENSIONAL_EXPONENTS(0.0,0.0,0.0,0.0,0.0,0.0,0.0);\n", idDegCtx);
    int idGlobUncert = id++;
    fprintf(f, "#%d=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#%d))GLOBAL_UNIT_ASSIGNED_CONTEXT((#%d,#%d,#%d))REPRESENTATION_CONTEXT('Context','#%d'));\n",
            idGlobUncert, idGlobUncert, idLenUnit, idUncert, idSolidAngUnit, idGlobUncert);
    int idOrigin = id++;
    fprintf(f, "#%d=CARTESIAN_POINT('',(0.0,0.0,0.0));\n", idOrigin);
    int idAxisDir = id++;
    fprintf(f, "#%d=DIRECTION('',(0.0,0.0,1.0));\n", idAxisDir);
    int idAxis3d = id++;
    fprintf(f, "#%d=AXIS2_PLACEMENT_3D('',#%d,#%d,$);\n", idAxis3d, idOrigin, idAxisDir);
    int idShapeRep = id++;
    fprintf(f, "#%d=SHAPE_REPRESENTATION('',(#%d),#%d);\n", idShapeRep, idAxis3d, idGlobUncert);

    // 为每个三角形生成面
    vector<int> faceIds;
    faceIds.reserve(m.tris.size());

    for (size_t i = 0; i < m.tris.size(); ++i) {
        const Triangle& tri = m.tris[i];
        double ax = tri.v[0].x * scale, ay = tri.v[0].y * scale, az = tri.v[0].z * scale;
        double bx = tri.v[1].x * scale, by = tri.v[1].y * scale, bz = tri.v[1].z * scale;
        double cx = tri.v[2].x * scale, cy = tri.v[2].y * scale, cz = tri.v[2].z * scale;

        // 3 个顶点
        int pa = id++; fprintf(f, "#%d=CARTESIAN_POINT('',(%.10f,%.10f,%.10f));\n", pa, ax, ay, az);
        int pb = id++; fprintf(f, "#%d=CARTESIAN_POINT('',(%.10f,%.10f,%.10f));\n", pb, bx, by, bz);
        int pc = id++; fprintf(f, "#%d=CARTESIAN_POINT('',(%.10f,%.10f,%.10f));\n", pc, cx, cy, cz);

        // 3 个 Vertex
        int va = id++; fprintf(f, "#%d=VERTEX_POINT('',#%d);\n", va, pa);
        int vb = id++; fprintf(f, "#%d=VERTEX_POINT('',#%d);\n", vb, pb);
        int vc = id++; fprintf(f, "#%d=VERTEX_POINT('',#%d);\n", vc, pc);

        // Edge 1: a->b
        double d1x = bx - ax, d1y = by - ay, d1z = bz - az;
        double l1 = sqrt(d1x * d1x + d1y * d1y + d1z * d1z);
        if (l1 < 1e-12) l1 = 1e-12;
        int dir1 = id++; fprintf(f, "#%d=DIRECTION('',(%.10f,%.10f,%.10f));\n", dir1, d1x / l1, d1y / l1, d1z / l1);
        int vec1 = id++; fprintf(f, "#%d=VECTOR('',#%d,%.10f);\n", vec1, dir1, l1);
        int line1 = id++; fprintf(f, "#%d=LINE('',#%d,#%d);\n", line1, pa, vec1);
        int ec1 = id++; fprintf(f, "#%d=EDGE_CURVE('',#%d,#%d,#%d,.T.);\n", ec1, va, vb, line1);
        int oe1 = id++; fprintf(f, "#%d=ORIENTED_EDGE('',*,.T.,#%d);\n", oe1, ec1);

        // Edge 2: b->c
        double d2x = cx - bx, d2y = cy - by, d2z = cz - bz;
        double l2 = sqrt(d2x * d2x + d2y * d2y + d2z * d2z);
        if (l2 < 1e-12) l2 = 1e-12;
        int dir2 = id++; fprintf(f, "#%d=DIRECTION('',(%.10f,%.10f,%.10f));\n", dir2, d2x / l2, d2y / l2, d2z / l2);
        int vec2 = id++; fprintf(f, "#%d=VECTOR('',#%d,%.10f);\n", vec2, dir2, l2);
        int line2 = id++; fprintf(f, "#%d=LINE('',#%d,#%d);\n", line2, pb, vec2);
        int ec2 = id++; fprintf(f, "#%d=EDGE_CURVE('',#%d,#%d,#%d,.T.);\n", ec2, vb, vc, line2);
        int oe2 = id++; fprintf(f, "#%d=ORIENTED_EDGE('',*,.T.,#%d);\n", oe2, ec2);

        // Edge 3: c->a
        double d3x = ax - cx, d3y = ay - cy, d3z = az - cz;
        double l3 = sqrt(d3x * d3x + d3y * d3y + d3z * d3z);
        if (l3 < 1e-12) l3 = 1e-12;
        int dir3 = id++; fprintf(f, "#%d=DIRECTION('',(%.10f,%.10f,%.10f));\n", dir3, d3x / l3, d3y / l3, d3z / l3);
        int vec3 = id++; fprintf(f, "#%d=VECTOR('',#%d,%.10f);\n", vec3, dir3, l3);
        int line3 = id++; fprintf(f, "#%d=LINE('',#%d,#%d);\n", line3, pc, vec3);
        int ec3 = id++; fprintf(f, "#%d=EDGE_CURVE('',#%d,#%d,#%d,.T.);\n", ec3, vc, va, line3);
        int oe3 = id++; fprintf(f, "#%d=ORIENTED_EDGE('',*,.T.,#%d);\n", oe3, ec3);

        // Edge loop + face bound
        int el = id++; fprintf(f, "#%d=EDGE_LOOP('',(#%d,#%d,#%d));\n", el, oe1, oe2, oe3);
        int fb = id++; fprintf(f, "#%d=FACE_OUTER_BOUND('',#%d,.T.);\n", fb, el);

        // Plane (法向量 + 面上一点)
        double nx = tri.n.x, ny = tri.n.y, nz = tri.n.z;
        double nl = sqrt(nx * nx + ny * ny + nz * nz);
        if (nl < 1e-12) { nx = 0; ny = 0; nz = 1; }
        else { nx /= nl; ny /= nl; nz /= nl; }
        int pdir = id++; fprintf(f, "#%d=DIRECTION('',(%.10f,%.10f,%.10f));\n", pdir, nx, ny, nz);
        int ploc = id++; fprintf(f, "#%d=CARTESIAN_POINT('',(%.10f,%.10f,%.10f));\n", ploc, ax, ay, az);
        int ax1 = id++; fprintf(f, "#%d=AXIS1_PLACEMENT('',#%d,#%d);\n", ax1, ploc, pdir);
        int plane = id++; fprintf(f, "#%d=PLANE('',#%d);\n", plane, ax1);

        // Advanced face
        int face = id++; fprintf(f, "#%d=ADVANCED_FACE('',(#%d),#%d,.T.);\n", face, fb, plane);
        faceIds.push_back(face);
    }

    // Closed shell
    int shellId = id++;
    fprintf(f, "#%d=CLOSED_SHELL('',(", shellId);
    for (size_t i = 0; i < faceIds.size(); ++i) {
        fprintf(f, "#%d", faceIds[i]);
        if (i + 1 < faceIds.size()) fprintf(f, ",");
    }
    fprintf(f, "));\n");

    // Manifold solid brep
    int solidId = id++;
    fprintf(f, "#%d=MANIFOLD_SOLID_BREP('',#%d);\n", solidId, shellId);

    // Shape representation of the solid
    int solidRep = id++;
    fprintf(f, "#%d=SHAPE_REPRESENTATION('',(#%d),#%d);\n", solidRep, idAxis3d, idGlobUncert);
    int shapeDef = id++;
    fprintf(f, "#%d=PRODUCT_DEFINITION_SHAPE('','',#%d);\n", shapeDef, shapeDef + 4);
    int shapeProp = id++;
    fprintf(f, "#%d=SHAPE_DEFINITION_REPRESENTATION(#%d,#%d);\n", shapeProp, shapeDef, solidRep);

    // Product context
    int prodCtx = id++;
    fprintf(f, "#%d=PRODUCT_CONTEXT('',#%d,'mechanical');\n", prodCtx, idGlobUncert);
    int prodDefCtx = id++;
    fprintf(f, "#%d=PRODUCT_DEFINITION_CONTEXT('part definition',#%d,'design');\n", prodDefCtx, idGlobUncert);
    int prod = id++;
    fprintf(f, "#%d=PRODUCT('gear','gear','gear',(#%d));\n", prod, prodCtx);
    int prodDef = id++;
    fprintf(f, "#%d=PRODUCT_DEFINITION('design','',#%d,#%d);\n", prodDef, prod, prodDefCtx);

    fprintf(f, "ENDSEC;\n");
    fprintf(f, "END-ISO-10303-21;\n");
    fclose(f);
    return true;
}

// ---------------- 多语言文本 ----------------
enum StrKey {
    S_WINDOW_TITLE, S_TITLE, S_LANG, S_LANG_ZH, S_LANG_EN,
    S_GEARTYPE, S_TYPE_SPUR, S_TYPE_CROWNED, S_TYPE_CROWNWHEEL,
    S_DIA, S_TEETH, S_THICK, S_SHEET, S_TOOTHH, S_FILLET, S_UNIT,
    S_UNIT_MM, S_UNIT_CM, S_UNIT_INCH, S_UNIT_M,
    S_BOSS, S_BOSS_THICK, S_BOSS_DIA, S_HOLE, S_HOLE_DIA, S_HOLE_SHAPE_CHK, S_HOLE_SHAPE,
    S_SHAPE_D, S_SHAPE_SQ, S_SHAPE_POLY, S_SHAPE_STAR, S_POLY_SIDES, S_STAR_POINTS,
    S_LEN, S_WID, S_LOCK, S_GEN, S_STATUS_IDLE, S_PREVIEW_TITLE, S_PREVIEW_PLACEHOLDER,
    S_MSGBOX_TITLE,
    S_E_DIA, S_E_TEETH, S_E_THICK, S_E_BOSS_THICK, S_E_BOSS_DIA, S_E_BOSS_BIG,
    S_E_BOSS_CROWN, S_E_HOLE_DIA, S_E_HOLE_D, S_E_HOLE_D_RATIO, S_E_HOLE_SIDES, S_E_HOLE_BIG,
    S_E_BOSS_WORD, S_E_ROOT_WORD, S_E_FILLET, S_E_FILLET_BIG, S_E_SHEET, S_E_TOOTHH,
    S_SUCCESS, S_WRITE_FAIL, S_FILTER_STL, S_FILTER_ALL, S_DEFAULT_FILE,
    S_EXPORT_FMT, S_FMT_STL, S_FMT_STEP, S_FILTER_STEP, S_DEFAULT_FILE_STEP,
    // 多层齿轮
    S_TYPE_STACKED, S_LAYER_COUNT, S_LAYER_ADD, S_LAYER_DEL,
    S_COL_DIA, S_COL_TEETH, S_COL_THICK, S_COL_BOSS_T, S_COL_BOSS_D,
    S_E_LAYER_EMPTY, S_E_LAYER_DIA, S_E_LAYER_TEETH, S_E_LAYER_THICK,
    S_SUCCESS_STACKED, S_COL_TYPE, S_COL_HEIGHT, S_DLG_TITLE, S_DLG_OK, S_DLG_CANCEL, S_SHAPE_CIRCLE,
    // 内齿轮 + 倒角
    S_TYPE_INTERNAL, S_CHAMFER, S_CHAMFER_SIZE, S_CHAMFER_LOC,
    S_CHAMFER_NONE, S_CHAMFER_TOP, S_CHAMFER_BOTTOM, S_CHAMFER_BOTH, S_CHAMFER_TIP,
    S_E_CHAMFER, S_DIA_INT,
    S_STR_COUNT
};

static const wchar_t* g_zh[S_STR_COUNT] = {
    L"简单齿轮生成器", L"齿轮参数配置（单位：毫米）", L"语言|language", L"中文", L"English",
    L"齿轮类型", L"普通齿轮", L"冠状轮（桶状）", L"垂直轮（冠齿轮）",
    L"外径（齿顶圆直径）：", L"齿数：", L"厚度：", L"薄片厚度：", L"齿高：", L"桶形量：", L"输出单位：",
    L"毫米 (mm)", L"厘米 (cm)", L"英寸 (inch)", L"米 (m)",
    L"添加中心凸起（凸台）", L"凸起厚度：", L"凸起直径：", L"添加中心孔", L"孔直径：", L"自定义孔形状", L"孔形状：",
    L"D 型口", L"方形口", L"多边形孔", L"星形孔", L"多边形边数：", L"星形角数：",
    L"长：", L"宽：", L"锁定长宽比", L"生成 STL 文件", L"请在左侧设置参数，右侧实时预览", L"实时预览", L"预览",
    L"简单齿轮生成器",
    L"外径必须大于 0", L"齿数至少为 5", L"厚度必须大于 0",
    L"凸起厚度必须大于 0", L"凸起直径必须大于 0",
    L"凸起直径(%.2f)不能大于等于最大允许直径(%.2f)，请减小凸起直径",
    L"凸起直径需小于 齿根圆直径 - 2×圆角半径",
    L"孔直径必须大于 0", L"D 型孔的长和宽都必须大于 0",
    L"D 型孔的宽需满足：长的一半 < 宽 < 长", L"多边形边数 / 星形角数至少为 3",
    L"孔尺寸过大：孔最大外接直径(%.2f)必须小于%ls直径(%.2f)",
    L"凸起", L"齿根圆",
    L"桶形量必须大于 0", L"桶形量过大：须小于 1.1×模数",
    L"薄片厚度必须大于 0", L"齿高必须大于 0",
    L"生成成功：齿数=%d，模数=%.3f mm，三角面=%d，输出单位=%ls",
    L"无法写入文件，请检查路径或文件权限",
    L"STL 文件 (*.stl)", L"所有文件 (*.*)", L"gear.stl",
    L"导出格式：", L"STL (二进制)", L"STEP (AP214)", L"STEP 文件 (*.step)", L"gear.step",
    // 多层齿轮
    L"多层齿轮（堆叠）", L"层数：", L"添加层", L"删除层",
    L"外径", L"齿数", L"厚度", L"凸起厚度", L"凸起直径",
    L"请至少添加一个齿轮层", L"第%d层外径必须大于0", L"第%d层齿数至少为5", L"第%d层厚度必须大于0",
    L"生成成功：层数=%d，三角面=%d，输出单位=%ls",
    L"类型", L"高度", L"编辑齿轮层", L"确定", L"取消", L"圆形",
    // 内齿轮 + 倒角
    L"内齿轮", L"倒角", L"倒角大小：", L"倒角位置：",
    L"无", L"顶边", L"底边", L"顶底边", L"齿尖",
    L"倒角大小必须大于 0", L"外径（圆环外径）："
};

static const wchar_t* g_en[S_STR_COUNT] = {
    L"Simple Gear Generator", L"Gear Parameters (units: mm)", L"语言|language", L"中文", L"English",
    L"Gear Type", L"Spur Gear", L"Crowned Gear (barrel)", L"Crown Wheel",
    L"Outer diameter:", L"Teeth:", L"Thickness:", L"Sheet thickness:", L"Tooth height:", L"Barrel amount:", L"Output unit:",
    L"mm (millimeter)", L"cm (centimeter)", L"inch", L"m (meter)",
    L"Add center boss", L"Boss thickness:", L"Boss diameter:", L"Add center hole", L"Hole diameter:", L"Custom hole shape", L"Hole shape:",
    L"D-shape", L"Square", L"Polygon", L"Star", L"Polygon sides:", L"Star points:",
    L"Length:", L"Width:", L"Lock ratio", L"Generate STL", L"Set parameters on the left; live preview on the right", L"Live Preview", L"Preview",
    L"Simple Gear Generator",
    L"Outer diameter must be > 0", L"Teeth must be at least 5", L"Thickness must be > 0",
    L"Boss thickness must be > 0", L"Boss diameter must be > 0",
    L"Boss diameter (%.2f) must be smaller than max allowed (%.2f)",
    L"Boss diameter must be < root diameter - 2×fillet radius",
    L"Hole diameter must be > 0", L"D-shape length and width must be > 0",
    L"D-shape width must satisfy: half length < width < length", L"Polygon sides / star points must be at least 3",
    L"Hole too large: max circumscribed diameter (%.2f) must be < %ls diameter (%.2f)",
    L"boss", L"root",
    L"Barrel amount must be > 0", L"Barrel amount too large: must be < 1.1×module",
    L"Sheet thickness must be > 0", L"Tooth height must be > 0",
    L"Generated: teeth=%d, module=%.3f mm, triangles=%d, unit=%ls",
    L"Cannot write file; check path or permissions",
    L"STL files (*.stl)", L"All files (*.*)", L"gear.stl",
    L"Export format:", L"STL (binary)", L"STEP (AP214)", L"STEP files (*.step)", L"gear.step",
    // Stacked gear
    L"Stacked Gear", L"Layer count:", L"Add Layer", L"Remove Layer",
    L"Outer Dia", L"Teeth", L"Thick", L"Boss Thick", L"Boss Dia",
    L"Add at least one gear layer", L"Layer %d: outer diameter must be > 0",
    L"Layer %d: teeth must be at least 5", L"Layer %d: thickness must be > 0",
    L"Generated: layers=%d, triangles=%d, unit=%ls",
    L"Type", L"Height", L"Edit Gear Layer", L"OK", L"Cancel", L"Circle",
    // Internal gear + Chamfer
    L"Internal Gear", L"Chamfer", L"Chamfer size:", L"Chamfer location:",
    L"None", L"Top edge", L"Bottom edge", L"Top & bottom", L"Tooth tips",
    L"Chamfer size must be > 0", L"Outer diameter (ring):"
};

enum Lang { LANG_ZH = 0, LANG_EN = 1 };
static Lang g_lang = LANG_ZH;

static const wchar_t* tr(int k) {
    return (g_lang == LANG_EN) ? g_en[k] : g_zh[k];
}

// ---------------- GUI ----------------
enum {
    IDC_GEARTYPE = 1001, IDC_LANG,
    IDC_DIA, IDC_TEETH, IDC_THICK, IDC_UNIT,
    IDC_BOSS_CHK, IDC_BOSS_THICK, IDC_BOSS_DIA,
    IDC_HOLE_CHK, IDC_HOLE_DIA, IDC_HOLE_SHAPE_CHK, IDC_HOLE_SHAPE_COMBO,
    IDC_HOLE_PARAM, IDC_HOLE_LEN, IDC_HOLE_WID, IDC_HOLE_LOCK,
    IDC_FILLET, IDC_SHEET, IDC_TOOTH_HEIGHT,
    IDC_GEN, IDC_STATUS, IDC_PREVIEW,
    IDC_EXPORT_FMT,
    // 多层齿轮
    IDC_LAYER_COUNT, IDC_LAYER_LIST, IDC_LAYER_ADD, IDC_LAYER_DEL,
    // 层编辑对话框
    IDC_DLG_TYPE, IDC_DLG_DIA, IDC_DLG_TEETH, IDC_DLG_THICK,
    IDC_DLG_SHEET, IDC_DLG_TOOTHH, IDC_DLG_FILLET,
    IDC_DLG_BOSS, IDC_DLG_BOSS_T, IDC_DLG_BOSS_D,
    IDC_DLG_LBL_THICK, IDC_DLG_LBL_SHEET, IDC_DLG_LBL_TOOTHH, IDC_DLG_LBL_FILLET,
    IDC_DLG_HOLE, IDC_DLG_HOLE_DIA, IDC_DLG_HOLE_SHAPE,
    // 倒角控件
    IDC_CHAMFER_CHK, IDC_CHAMFER_SIZE, IDC_CHAMFER_LOC,
    IDC_DLG_CHAMFER_CHK, IDC_DLG_CHAMFER_SIZE, IDC_DLG_CHAMFER_LOC
};

static HINSTANCE g_hInst;
static HWND g_hwnd;
static HWND g_edDia, g_edTeeth, g_edThick, g_cmbUnit;
static HWND g_chkBoss, g_edBossThick, g_edBossDia;
static HWND g_chkHole, g_edHoleDia;
static HWND g_chkShape, g_cmbShape, g_lblParam, g_edParam;
static HWND g_lblLen, g_edLen, g_lblWid, g_edWid, g_chkLock;
static HWND g_btnGen, g_stStatus;
static HWND g_cmbGear, g_cmbLang;
static HWND g_edFillet, g_edSheet, g_edToothH;
static HWND g_lblLang, g_lblGear, g_lblDia, g_lblTeeth, g_lblThick, g_lblSheet, g_lblToothH, g_lblFillet, g_lblUnit;
static HWND g_lblBossThick, g_lblBossDia, g_lblHoleDia, g_lblShape, g_lblPreviewTitle;
static HWND g_lblExportFmt, g_cmbExportFmt;
static HWND g_chkChamfer, g_edChamferSize, g_cmbChamferLoc;
static HWND g_lblChamferSize, g_lblChamferLoc;
static HWND g_preview;
static HFONT g_font;
static GearType g_gearType = GT_SPUR;
static Mesh g_previewMesh;
static double g_yaw = 0.6;    // 预览旋转角（绕 Z）
static double g_pitch = 0.45; // 预览俯仰角（绕 X）
static double g_zoom = 1.0;  // 预览缩放因子
static POINT g_dragLast;
static int g_genBtnBottom = 0; // 生成按钮底部 Y 坐标，用于日志窗口布局
static HMODULE g_richMod = nullptr;       // RichEdit DLL 句柄
static const wchar_t* g_richClass = L"EDIT"; // RichEdit 窗口类名（回退为 EDIT）

static double g_lockRatio = 0.8;
static bool g_lockEditing = false;

// 多层齿轮
static HWND g_lblLayerCount, g_edLayerCount, g_lvLayers, g_btnAddLayer, g_btnDelLayer;
static vector<LayerInfo> g_layers;        // 各层信息
static bool g_layerSyncing = false;       // 防止 ListView ↔ g_layers 循环刷新
static LayerInfo g_editLayer;              // 正在编辑的层（对话框用）
static int      g_editLayerIdx = -1;       // 正在编辑的层索引

static double unitScale(int sel) {
    switch (sel) {
    case 0: return 1.0;
    case 1: return 0.1;
    case 2: return 1.0 / 25.4;
    default: return 0.001;
    }
}

// 加载 RichEdit DLL，优先使用 msftedit（5.0），回退到 riched20（2.0）
static void initRichEdit() {
    g_richMod = LoadLibraryW(L"msftedit.dll");
    if (g_richMod) { g_richClass = MSFTEDIT_CLASS; return; }
    g_richMod = LoadLibraryW(L"riched20.dll");
    if (g_richMod) { g_richClass = RICHEDIT_CLASS; return; }
    g_richClass = L"EDIT"; // 无 RichEdit 时回退到普通 EDIT
}

static HFONT makeFont(int size, bool bold) {
    return CreateFontW(-size, 0, 0, 0, bold ? FW_BOLD : FW_NORMAL, FALSE, FALSE, FALSE,
                       DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY,
                       DEFAULT_PITCH | FF_DONTCARE, L"Microsoft YaHei UI");
}

static HWND makeLabel(HWND parent, const wchar_t* text, int x, int y, int w, int h) {
    HWND c = CreateWindowW(L"STATIC", text, WS_CHILD | WS_VISIBLE, x, y, w, h, parent, nullptr, g_hInst, nullptr);
    SendMessageW(c, WM_SETFONT, (WPARAM)g_font, TRUE);
    return c;
}
static HWND makeEdit(HWND parent, int id, int x, int y, int w, int h) {
    HWND c = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"", WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL,
                             x, y, w, h, parent, (HMENU)(INT_PTR)id, g_hInst, nullptr);
    SendMessageW(c, WM_SETFONT, (WPARAM)g_font, TRUE);
    return c;
}
static HWND makeCheck(HWND parent, int id, const wchar_t* text, int x, int y, int w, int h) {
    HWND c = CreateWindowW(L"BUTTON", text, WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX,
                           x, y, w, h, parent, (HMENU)(INT_PTR)id, g_hInst, nullptr);
    SendMessageW(c, WM_SETFONT, (WPARAM)g_font, TRUE);
    return c;
}
static HWND makeCombo(HWND parent, int id, int x, int y, int w, int h) {
    HWND c = CreateWindowW(L"COMBOBOX", L"", WS_CHILD | WS_VISIBLE | WS_TABSTOP | CBS_DROPDOWNLIST,
                           x, y, w, h, parent, (HMENU)(INT_PTR)id, g_hInst, nullptr);
    SendMessageW(c, WM_SETFONT, (WPARAM)g_font, TRUE);
    return c;
}

static double readDouble(HWND ed) {
    wchar_t buf[128];
    GetWindowTextW(ed, buf, 128);
    wstring s = buf;
    for (auto& ch : s) { if (ch == L'，') ch = L'.'; if (ch == L'．') ch = L'.'; }
    return wcstod(s.c_str(), nullptr);
}
static long readLong(HWND ed) {
    wchar_t buf[128];
    GetWindowTextW(ed, buf, 128);
    return wcstol(buf, nullptr, 10);
}
static bool isChecked(HWND h) { return SendMessageW(h, BM_GETCHECK, 0, 0) == BST_CHECKED; }
static int comboSel(HWND h) { return (int)SendMessageW(h, CB_GETCURSEL, 0, 0); }
static void msgBox(HWND hwnd, const wchar_t* text) {
    MessageBoxW(hwnd, text, tr(S_MSGBOX_TITLE), MB_OK | MB_ICONINFORMATION);
}

// 根据导出格式动态更新生成按钮文字
static void updateGenButtonText() {
    int fmtSel = comboSel(g_cmbExportFmt);
    bool isStep = (fmtSel == 1);
    const wchar_t* fmt = isStep ? L"STEP" : L"STL";
    wchar_t buf[64];
    if (g_lang == LANG_EN) swprintf(buf, 64, L"Generate %ls", fmt);
    else swprintf(buf, 64, L"生成 %ls 文件", fmt);
    if (g_btnGen) SetWindowTextW(g_btnGen, buf);
}

// 日志核心：向 RichEdit 追加一行，普通行浅绿、错误行红色
static void logAppend(bool isError, const wchar_t* fmt, va_list args) {
    if (!g_stStatus) return;
    SYSTEMTIME st;
    GetLocalTime(&st);
    wchar_t timeBuf[32];
    swprintf(timeBuf, 32, L"[%02d:%02d:%02d] ", st.wHour, st.wMinute, st.wSecond);
    wchar_t msg[512];
    _vsnwprintf(msg, 511, fmt, args);
    msg[511] = L'\0';
    // 错误行格式: [HH:MM:SS](>_<)!1>> message ；普通行: [HH:MM:SS] message
    wchar_t line[700];
    if (isError)
        swprintf(line, 700, L"%ls(>_<)!1>> %ls\r\n", timeBuf, msg);
    else
        swprintf(line, 700, L"%ls%ls\r\n", timeBuf, msg);
    // 设置插入点颜色后再写入
    CHARFORMAT2 cf;
    ZeroMemory(&cf, sizeof(cf));
    cf.cbSize = sizeof(cf);
    cf.dwMask = CFM_COLOR;
    cf.crTextColor = isError ? RGB(255, 85, 85) : RGB(180, 230, 180);
    int len = GetWindowTextLengthW(g_stStatus);
    SendMessageW(g_stStatus, EM_SETSEL, len, len);
    SendMessageW(g_stStatus, EM_SETCHARFORMAT, SCF_SELECTION, (LPARAM)&cf);
    SendMessageW(g_stStatus, EM_REPLACESEL, FALSE, (LPARAM)line);
    SendMessageW(g_stStatus, EM_SCROLLCARET, 0, 0);
}

// 普通日志（浅绿色）
static void logMsg(const wchar_t* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    logAppend(false, fmt, args);
    va_end(args);
}

// 错误日志（红色）
static void logErr(const wchar_t* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    logAppend(true, fmt, args);
    va_end(args);
}

static void syncLenWid(bool fromLen) {
    if (g_lockEditing) return;
    if (!isChecked(g_chkLock)) return;
    double L = readDouble(g_edLen);
    double W = readDouble(g_edWid);
    g_lockEditing = true;
    wchar_t buf[64];
    if (fromLen) {
        swprintf(buf, 64, L"%.4f", L * g_lockRatio);
        SetWindowTextW(g_edWid, buf);
    } else {
        double newL = (g_lockRatio > 0) ? W / g_lockRatio : W;
        swprintf(buf, 64, L"%.4f", newL);
        SetWindowTextW(g_edLen, buf);
    }
    g_lockEditing = false;
}

static void updateHoleControls() {
    BOOL holeOn = isChecked(g_chkHole);
    BOOL shapeOn = holeOn && isChecked(g_chkShape);
    int sel = comboSel(g_cmbShape);

    bool holeDiaUsed = holeOn && (!shapeOn || sel != 0);
    EnableWindow(g_edHoleDia, holeDiaUsed);
    EnableWindow(g_chkShape, holeOn);
    EnableWindow(g_cmbShape, shapeOn);

    bool showD = shapeOn && sel == 0;
    ShowWindow(g_lblLen, showD ? SW_SHOW : SW_HIDE);
    ShowWindow(g_edLen, showD ? SW_SHOW : SW_HIDE);
    ShowWindow(g_lblWid, showD ? SW_SHOW : SW_HIDE);
    ShowWindow(g_edWid, showD ? SW_SHOW : SW_HIDE);
    ShowWindow(g_chkLock, showD ? SW_SHOW : SW_HIDE);

    bool showParam = shapeOn && (sel == 2 || sel == 3);
    SetWindowTextW(g_lblParam, (sel == 2) ? tr(S_POLY_SIDES) : tr(S_STAR_POINTS));
    ShowWindow(g_lblParam, showParam ? SW_SHOW : SW_HIDE);
    ShowWindow(g_edParam, showParam ? SW_SHOW : SW_HIDE);
}

// ---------------- 多层齿轮 ListView + 对话框 辅助函数 ----------------

static void rebuildPreview();  // 前向声明

// 层类型名称
static const wchar_t* layerTypeStr(GearType t) {
    switch (t) {
    case GT_CROWNED:    return tr(S_TYPE_CROWNED);
    case GT_CROWNWHEEL: return tr(S_TYPE_CROWNWHEEL);
    case GT_INTERNAL:   return tr(S_TYPE_INTERNAL);
    default:            return tr(S_TYPE_SPUR);
    }
}

// 初始化默认层
static void initDefaultLayers() {
    if (!g_layers.empty()) return;
    GearParams p1 = {};
    p1.outerDiameter = 40; p1.teeth = 20; p1.thickness = 5;
    p1.hasBoss = true; p1.bossThickness = 5; p1.bossDiameter = 20;
    g_layers.push_back({ GT_SPUR, p1 });
    GearParams p2 = {};
    p2.outerDiameter = 30; p2.teeth = 15; p2.thickness = 4;
    g_layers.push_back({ GT_SPUR, p2 });
}

// 更新层数编辑框（不触发回调）
static void updateLayerCountField() {
    if (!g_edLayerCount) return;
    g_layerSyncing = true;
    wchar_t buf[16];
    swprintf(buf, 16, L"%d", (int)g_layers.size());
    SetWindowTextW(g_edLayerCount, buf);
    g_layerSyncing = false;
}

// 设置 ListView 列头（语言切换时调用）
static void updateLayerListColumns() {
    if (!g_lvLayers) return;
    int cols[] = { 36, 100, 76, 54, 76 };
    const wchar_t* hdrs[] = {
        L"#", tr(S_COL_TYPE), tr(S_COL_DIA), tr(S_COL_TEETH), tr(S_COL_HEIGHT)
    };
    while (ListView_DeleteColumn(g_lvLayers, 1))
        ;
    for (int i = 0; i < 5; ++i) {
        LVCOLUMNW col;
        col.mask = LVCF_TEXT | LVCF_WIDTH | LVCF_FMT;
        col.fmt = LVCFMT_CENTER;
        col.cx = cols[i];
        col.pszText = (LPWSTR)hdrs[i];
        ListView_InsertColumn(g_lvLayers, i, &col);
    }
}

// 从 g_layers 刷新 ListView 全部行
static void refreshLayerList() {
    if (!g_lvLayers) return;
    g_layerSyncing = true;
    SendMessageW(g_lvLayers, WM_SETREDRAW, FALSE, 0);
    ListView_DeleteAllItems(g_lvLayers);
    for (size_t i = 0; i < g_layers.size(); ++i) {
        wchar_t idx[8], dia[32], teeth[8], hgt[32];
        swprintf(idx, 8, L"%d", (int)(i + 1));
        swprintf(dia, 32, L"%.2f", g_layers[i].params.outerDiameter);
        swprintf(teeth, 8, L"%d", g_layers[i].params.teeth);
        swprintf(hgt, 32, L"%.2f", layerHeight(g_layers[i]));

        LVITEMW item;
        item.mask = LVIF_TEXT;
        item.iItem = (int)i;
        item.iSubItem = 0;
        item.pszText = idx;
        ListView_InsertItem(g_lvLayers, &item);
        ListView_SetItemText(g_lvLayers, (int)i, 1, (LPWSTR)layerTypeStr(g_layers[i].type));
        ListView_SetItemText(g_lvLayers, (int)i, 2, dia);
        ListView_SetItemText(g_lvLayers, (int)i, 3, teeth);
        ListView_SetItemText(g_lvLayers, (int)i, 4, hgt);
    }
    SendMessageW(g_lvLayers, WM_SETREDRAW, TRUE, 0);
    InvalidateRect(g_lvLayers, NULL, FALSE);
    updateLayerCountField();
    g_layerSyncing = false;
}

// 校验多层参数
static bool validateLayerParams(HWND hwnd, bool show) {
    if (g_layers.empty()) {
        if (show) msgBox(hwnd, tr(S_E_LAYER_EMPTY));
        return false;
    }
    for (size_t i = 0; i < g_layers.size(); ++i) {
        const GearParams& p = g_layers[i].params;
        if (p.outerDiameter <= 0) {
            if (show) { wchar_t b[256]; swprintf(b, 256, tr(S_E_LAYER_DIA), (int)(i+1)); msgBox(hwnd, b); }
            return false;
        }
        if (p.teeth < 5) {
            if (show) { wchar_t b[256]; swprintf(b, 256, tr(S_E_LAYER_TEETH), (int)(i+1)); msgBox(hwnd, b); }
            return false;
        }
        if (g_layers[i].type == GT_CROWNWHEEL) {
            if (p.sheetThickness <= 0 || p.toothHeight <= 0) {
                if (show) { wchar_t b[256]; swprintf(b, 256, tr(S_E_LAYER_THICK), (int)(i+1)); msgBox(hwnd, b); }
                return false;
            }
        } else {
            if (p.thickness <= 0) {
                if (show) { wchar_t b[256]; swprintf(b, 256, tr(S_E_LAYER_THICK), (int)(i+1)); msgBox(hwnd, b); }
                return false;
            }
        }
    }
    return true;
}

// ---------------- 层编辑对话框 ----------------

// 内存对话框模板构建器
struct DlgBuilder {
    vector<BYTE> data;
    int itemCount = 0;
    size_t cditOffset = 0;

    void align() { while (data.size() % 4) data.push_back(0); }
    void putW(WORD w) { data.push_back((BYTE)(w & 0xFF)); data.push_back((BYTE)((w >> 8) & 0xFF)); }
    void putD(DWORD d) { putW((WORD)(d & 0xFFFF)); putW((WORD)((d >> 16) & 0xFFFF)); }
    void putStr(const wchar_t* s) { for (; *s; ++s) putW(*s); putW(0); }

    void begin(DWORD style, int x, int y, int cx, int cy, const wchar_t* title, int fontsize, const wchar_t* font) {
        align();
        putD(style);
        putD(0);  // exStyle
        cditOffset = data.size();
        putW(0);  // cdit placeholder
        putW((WORD)x); putW((WORD)y); putW((WORD)cx); putW((WORD)cy);
        putW(0);  // menu: none
        putW(0);  // class: default
        putStr(title);
        putW((WORD)fontsize);
        putStr(font);
    }

    void addItem(DWORD style, DWORD exStyle, int x, int y, int cx, int cy, WORD id, WORD cls, const wchar_t* title) {
        align();
        putD(style);
        putD(exStyle);
        putW((WORD)x); putW((WORD)y); putW((WORD)cx); putW((WORD)cy);
        putW(id);
        putW(0xFFFF);  // predefined class
        putW(cls);
        if (title) putStr(title); else putW(0);
        putW(0);  // creation data
        itemCount++;
    }

    DLGTEMPLATE* finish() {
        *(WORD*)(data.data() + cditOffset) = (WORD)itemCount;
        return (DLGTEMPLATE*)data.data();
    }
};

// 对话框内根据齿轮类型 显示/隐藏 控件
static void updateDlgControls(HWND hDlg) {
    int sel = (int)SendDlgItemMessageW(hDlg, IDC_DLG_TYPE, CB_GETCURSEL, 0, 0);
    bool crownWheel = (sel == (int)GT_CROWNWHEEL);
    bool crowned = (sel == (int)GT_CROWNED);
    bool internal = (sel == (int)GT_INTERNAL);

    // 厚度（普通齿轮 / 内齿轮）/ 薄片厚度 + 齿高（冠齿轮）/ 桶形量（桶形齿轮）
    auto show = [&](int lblId, int edId, bool v) {
        ShowWindow(GetDlgItem(hDlg, lblId), v ? SW_SHOW : SW_HIDE);
        ShowWindow(GetDlgItem(hDlg, edId),   v ? SW_SHOW : SW_HIDE);
    };
    show(IDC_DLG_LBL_THICK,  IDC_DLG_THICK,  !crownWheel);
    show(IDC_DLG_LBL_SHEET,  IDC_DLG_SHEET,   crownWheel);
    show(IDC_DLG_LBL_TOOTHH, IDC_DLG_TOOTHH,  crownWheel);
    show(IDC_DLG_LBL_FILLET, IDC_DLG_FILLET,  crowned);

    // 内齿轮无凸起/孔
    ShowWindow(GetDlgItem(hDlg, IDC_DLG_BOSS),   !internal ? SW_SHOW : SW_HIDE);
    ShowWindow(GetDlgItem(hDlg, IDC_DLG_BOSS_T), !internal ? SW_SHOW : SW_HIDE);
    ShowWindow(GetDlgItem(hDlg, IDC_DLG_BOSS_D), !internal ? SW_SHOW : SW_HIDE);
    // BOSS label rows (static text without ID are skipped; they're always visible but harmless)
    ShowWindow(GetDlgItem(hDlg, IDC_DLG_HOLE),    !internal ? SW_SHOW : SW_HIDE);
    ShowWindow(GetDlgItem(hDlg, IDC_DLG_HOLE_DIA), !internal ? SW_SHOW : SW_HIDE);
    ShowWindow(GetDlgItem(hDlg, IDC_DLG_HOLE_SHAPE), !internal ? SW_SHOW : SW_HIDE);

    // 凸起控件启用/禁用
    BOOL bossOn = !internal && (IsDlgButtonChecked(hDlg, IDC_DLG_BOSS) == BST_CHECKED);
    EnableWindow(GetDlgItem(hDlg, IDC_DLG_BOSS_T), bossOn);
    EnableWindow(GetDlgItem(hDlg, IDC_DLG_BOSS_D), bossOn);

    // 中心孔控件启用/禁用
    BOOL holeOn = !internal && (IsDlgButtonChecked(hDlg, IDC_DLG_HOLE) == BST_CHECKED);
    EnableWindow(GetDlgItem(hDlg, IDC_DLG_HOLE_DIA), holeOn);
    EnableWindow(GetDlgItem(hDlg, IDC_DLG_HOLE_SHAPE), holeOn);

    // 倒角控件（普通齿轮和内齿轮可见）
    bool showChamfer = (sel == (int)GT_SPUR || sel == (int)GT_INTERNAL);
    ShowWindow(GetDlgItem(hDlg, IDC_DLG_CHAMFER_CHK),  showChamfer ? SW_SHOW : SW_HIDE);
    ShowWindow(GetDlgItem(hDlg, IDC_DLG_CHAMFER_SIZE), showChamfer ? SW_SHOW : SW_HIDE);
    ShowWindow(GetDlgItem(hDlg, IDC_DLG_CHAMFER_LOC),  showChamfer ? SW_SHOW : SW_HIDE);
    BOOL chamfOn = showChamfer && (IsDlgButtonChecked(hDlg, IDC_DLG_CHAMFER_CHK) == BST_CHECKED);
    EnableWindow(GetDlgItem(hDlg, IDC_DLG_CHAMFER_SIZE), chamfOn);
    EnableWindow(GetDlgItem(hDlg, IDC_DLG_CHAMFER_LOC),  chamfOn);
}

// 层编辑对话框过程
static INT_PTR CALLBACK LayerEditDlgProc(HWND hDlg, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_INITDIALOG: {
        // 齿轮类型下拉
        const wchar_t* types[] = { tr(S_TYPE_SPUR), tr(S_TYPE_CROWNED), tr(S_TYPE_CROWNWHEEL), tr(S_TYPE_INTERNAL) };
        for (int i = 0; i < 4; ++i)
            SendDlgItemMessageW(hDlg, IDC_DLG_TYPE, CB_ADDSTRING, 0, (LPARAM)types[i]);
        SendDlgItemMessageW(hDlg, IDC_DLG_TYPE, CB_SETCURSEL, (int)g_editLayer.type, 0);

        // 填入数值
        auto setVal = [&](int id, double v) {
            wchar_t b[32]; swprintf(b, 32, L"%.2f", v);
            SetDlgItemTextW(hDlg, id, b);
        };
        auto setInt = [&](int id, int v) {
            wchar_t b[32]; swprintf(b, 32, L"%d", v);
            SetDlgItemTextW(hDlg, id, b);
        };
        const GearParams& p = g_editLayer.params;
        setVal(IDC_DLG_DIA, p.outerDiameter);
        setInt(IDC_DLG_TEETH, p.teeth);
        setVal(IDC_DLG_THICK, p.thickness);
        setVal(IDC_DLG_SHEET, p.sheetThickness);
        setVal(IDC_DLG_TOOTHH, p.toothHeight);
        setVal(IDC_DLG_FILLET, p.filletRadius);
        CheckDlgButton(hDlg, IDC_DLG_BOSS, p.hasBoss ? BST_CHECKED : BST_UNCHECKED);
        setVal(IDC_DLG_BOSS_T, p.bossThickness);
        setVal(IDC_DLG_BOSS_D, p.bossDiameter);

        // 中心孔
        CheckDlgButton(hDlg, IDC_DLG_HOLE, p.hasHole ? BST_CHECKED : BST_UNCHECKED);
        setVal(IDC_DLG_HOLE_DIA, p.holeDiameter);
        const wchar_t* shapes[] = { tr(S_SHAPE_CIRCLE), tr(S_SHAPE_D), tr(S_SHAPE_SQ), tr(S_SHAPE_POLY), tr(S_SHAPE_STAR) };
        for (int i = 0; i < 5; ++i)
            SendDlgItemMessageW(hDlg, IDC_DLG_HOLE_SHAPE, CB_ADDSTRING, 0, (LPARAM)shapes[i]);
        SendDlgItemMessageW(hDlg, IDC_DLG_HOLE_SHAPE, CB_SETCURSEL, (int)p.holeShape, 0);

        // 倒角
        CheckDlgButton(hDlg, IDC_DLG_CHAMFER_CHK, p.hasChamfer ? BST_CHECKED : BST_UNCHECKED);
        setVal(IDC_DLG_CHAMFER_SIZE, p.chamferSize);
        const wchar_t* chamfLocs[] = { tr(S_CHAMFER_NONE), tr(S_CHAMFER_TOP), tr(S_CHAMFER_BOTTOM), tr(S_CHAMFER_BOTH), tr(S_CHAMFER_TIP) };
        for (int i = 0; i < 5; ++i)
            SendDlgItemMessageW(hDlg, IDC_DLG_CHAMFER_LOC, CB_ADDSTRING, 0, (LPARAM)chamfLocs[i]);
        SendDlgItemMessageW(hDlg, IDC_DLG_CHAMFER_LOC, CB_SETCURSEL, p.chamferLoc, 0);

        updateDlgControls(hDlg);
        return TRUE;
    }
    case WM_COMMAND: {
        WORD id = LOWORD(wp);
        WORD code = HIWORD(wp);
        if (id == IDC_DLG_TYPE && code == CBN_SELCHANGE) {
            updateDlgControls(hDlg);
            return TRUE;
        }
        if (id == IDC_DLG_BOSS && code == BN_CLICKED) {
            updateDlgControls(hDlg);
            return TRUE;
        }
        if (id == IDC_DLG_HOLE && code == BN_CLICKED) {
            updateDlgControls(hDlg);
            return TRUE;
        }
        if (id == IDC_DLG_CHAMFER_CHK && code == BN_CLICKED) {
            updateDlgControls(hDlg);
            return TRUE;
        }
        if (id == IDOK) {
            // 读回所有值
            auto getVal = [&](int id) -> double {
                wchar_t b[64]; GetDlgItemTextW(hDlg, id, b, 64);
                return wcslen(b) > 0 ? wcstod(b, nullptr) : 0.0;
            };
            auto getInt = [&](int id) -> int {
                wchar_t b[64]; GetDlgItemTextW(hDlg, id, b, 64);
                return wcslen(b) > 0 ? (int)wcstol(b, nullptr, 10) : 0;
            };
            g_editLayer.type  = (GearType)SendDlgItemMessageW(hDlg, IDC_DLG_TYPE, CB_GETCURSEL, 0, 0);
            GearParams& p = g_editLayer.params;
            p.outerDiameter = getVal(IDC_DLG_DIA);
            p.teeth         = getInt(IDC_DLG_TEETH);
            p.thickness     = getVal(IDC_DLG_THICK);
            p.sheetThickness = getVal(IDC_DLG_SHEET);
            p.toothHeight   = getVal(IDC_DLG_TOOTHH);
            p.filletRadius  = getVal(IDC_DLG_FILLET);
            p.hasBoss       = IsDlgButtonChecked(hDlg, IDC_DLG_BOSS) == BST_CHECKED;
            p.bossThickness = getVal(IDC_DLG_BOSS_T);
            p.bossDiameter  = getVal(IDC_DLG_BOSS_D);
            // 中心孔
            p.hasHole       = IsDlgButtonChecked(hDlg, IDC_DLG_HOLE) == BST_CHECKED;
            p.holeDiameter  = getVal(IDC_DLG_HOLE_DIA);
            int shapeSel    = (int)SendDlgItemMessageW(hDlg, IDC_DLG_HOLE_SHAPE, CB_GETCURSEL, 0, 0);
            p.holeShape     = (HoleShape)(shapeSel >= 0 ? shapeSel : 0);
            if (p.holeShape == HS_POLYGON)      p.holeSides = p.holeSides > 0 ? p.holeSides : 6;
            else if (p.holeShape == HS_STAR)   p.holeSides = p.holeSides > 0 ? p.holeSides : 5;
            else                                p.holeSides = 0;
            if (p.holeShape == HS_D) { p.dLength = p.dLength > 0 ? p.dLength : p.holeDiameter; p.dWidth = p.dWidth > 0 ? p.dWidth : p.holeDiameter * 0.6; }
            else { p.dLength = 0; p.dWidth = 0; }
            // 倒角
            p.hasChamfer  = IsDlgButtonChecked(hDlg, IDC_DLG_CHAMFER_CHK) == BST_CHECKED;
            p.chamferSize = getVal(IDC_DLG_CHAMFER_SIZE);
            int chamfSel  = (int)SendDlgItemMessageW(hDlg, IDC_DLG_CHAMFER_LOC, CB_GETCURSEL, 0, 0);
            p.chamferLoc  = chamfSel >= 0 ? chamfSel : (int)CL_NONE;
            EndDialog(hDlg, IDOK);
            return TRUE;
        }
        if (id == IDCANCEL) {
            EndDialog(hDlg, IDCANCEL);
            return TRUE;
        }
        break;
    }
    }
    return FALSE;
}

// 构建对话框模板并打开模态编辑窗口
static void showLayerEditDialog(int index) {
    if (index < 0 || index >= (int)g_layers.size()) return;

    // 保存正在编辑的层
    g_editLayer = g_layers[index];
    g_editLayerIdx = index;

    // 在内存中构建对话框模板
    DlgBuilder b;
    b.begin(WS_POPUP | WS_CAPTION | WS_SYSMENU | DS_CENTER | DS_SETFONT | DS_MODALFRAME,
           0, 0, 310, 380, tr(S_DLG_TITLE), 9, L"Microsoft YaHei UI");

    // 0xFFFF + class constants: 0x0080=Button, 0x0081=Edit, 0x0082=Static, 0x0085=ComboBox
    // Row 1: 齿轮类型
    b.addItem(WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 10, 10, 80, 14, -1, 0x0082, tr(S_GEARTYPE));
    b.addItem(WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL | WS_TABSTOP, 0, 95, 8, 195, 200,
              IDC_DLG_TYPE, 0x0085, nullptr);
    // Row 2: 外径
    b.addItem(WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 10, 32, 80, 14, -1, 0x0082, tr(S_DIA));
    b.addItem(WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL | WS_TABSTOP, 0, 95, 30, 100, 14,
              IDC_DLG_DIA, 0x0081, nullptr);
    // Row 3: 齿数
    b.addItem(WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 10, 54, 80, 14, -1, 0x0082, tr(S_TEETH));
    b.addItem(WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL | WS_TABSTOP, 0, 95, 52, 60, 14,
              IDC_DLG_TEETH, 0x0081, nullptr);
    // Row 4: 厚度 (spur / crowned)
    b.addItem(WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 10, 76, 80, 14, IDC_DLG_LBL_THICK, 0x0082, tr(S_THICK));
    b.addItem(WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL | WS_TABSTOP, 0, 95, 74, 100, 14,
              IDC_DLG_THICK, 0x0081, nullptr);
    // Row 5: 薄片厚度 (crown wheel)
    b.addItem(WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 10, 98, 80, 14, IDC_DLG_LBL_SHEET, 0x0082, tr(S_SHEET));
    b.addItem(WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL | WS_TABSTOP, 0, 95, 96, 100, 14,
              IDC_DLG_SHEET, 0x0081, nullptr);
    // Row 6: 齿高 (crown wheel)
    b.addItem(WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 10, 120, 80, 14, IDC_DLG_LBL_TOOTHH, 0x0082, tr(S_TOOTHH));
    b.addItem(WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL | WS_TABSTOP, 0, 95, 118, 100, 14,
              IDC_DLG_TOOTHH, 0x0081, nullptr);
    // Row 7: 桶形量 (crowned)
    b.addItem(WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 10, 142, 80, 14, IDC_DLG_LBL_FILLET, 0x0082, tr(S_FILLET));
    b.addItem(WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL | WS_TABSTOP, 0, 95, 140, 100, 14,
              IDC_DLG_FILLET, 0x0081, nullptr);
    // Row 8: 中心凸起
    b.addItem(WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX | WS_TABSTOP, 0, 10, 168, 200, 14,
              IDC_DLG_BOSS, 0x0080, tr(S_BOSS));
    // Row 9: 凸起厚度
    b.addItem(WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 30, 192, 70, 14, -1, 0x0082, tr(S_BOSS_THICK));
    b.addItem(WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL | WS_TABSTOP, 0, 105, 190, 80, 14,
              IDC_DLG_BOSS_T, 0x0081, nullptr);
    // Row 10: 凸起直径
    b.addItem(WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 30, 214, 70, 14, -1, 0x0082, tr(S_BOSS_DIA));
    b.addItem(WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL | WS_TABSTOP, 0, 105, 212, 80, 14,
              IDC_DLG_BOSS_D, 0x0081, nullptr);
    // Row 11: 中心孔
    b.addItem(WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX | WS_TABSTOP, 0, 10, 236, 120, 14,
              IDC_DLG_HOLE, 0x0080, tr(S_HOLE));
    b.addItem(WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 135, 236, 40, 14, -1, 0x0082, tr(S_HOLE_DIA));
    b.addItem(WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL | WS_TABSTOP, 0, 180, 234, 60, 14,
              IDC_DLG_HOLE_DIA, 0x0081, nullptr);
    // Row 12: 孔形状
    b.addItem(WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 10, 258, 80, 14, -1, 0x0082, tr(S_HOLE_SHAPE));
    b.addItem(WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL | WS_TABSTOP, 0, 95, 256, 195, 200,
              IDC_DLG_HOLE_SHAPE, 0x0085, nullptr);
    // Row 13: 倒角
    b.addItem(WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX | WS_TABSTOP, 0, 10, 280, 120, 14,
              IDC_DLG_CHAMFER_CHK, 0x0080, tr(S_CHAMFER));
    // Row 14: 倒角大小 + 位置
    b.addItem(WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 30, 304, 70, 14, -1, 0x0082, tr(S_CHAMFER_SIZE));
    b.addItem(WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL | WS_TABSTOP, 0, 105, 302, 50, 14,
              IDC_DLG_CHAMFER_SIZE, 0x0081, nullptr);
    b.addItem(WS_CHILD | WS_VISIBLE | SS_LEFT, 0, 165, 304, 50, 14, -1, 0x0082, tr(S_CHAMFER_LOC));
    b.addItem(WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL | WS_TABSTOP, 0, 220, 300, 80, 200,
              IDC_DLG_CHAMFER_LOC, 0x0085, nullptr);
    // Row 15: OK / Cancel
    b.addItem(WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON | WS_TABSTOP, 0, 60, 330, 80, 18,
              IDOK, 0x0080, tr(S_DLG_OK));
    b.addItem(WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_TABSTOP, 0, 170, 330, 80, 18,
              IDCANCEL, 0x0080, tr(S_DLG_CANCEL));

    INT_PTR ret = DialogBoxIndirectParamW(g_hInst, b.finish(), g_hwnd, LayerEditDlgProc, 0);
    if (ret == IDOK) {
        g_layers[g_editLayerIdx] = g_editLayer;
        refreshLayerList();
        rebuildPreview();
        logMsg(L"Layer %d updated: type=%d dia=%.2f teeth=%d",
               g_editLayerIdx + 1, (int)g_editLayer.type,
               g_editLayer.params.outerDiameter, g_editLayer.params.teeth);
    }
    g_editLayerIdx = -1;
}

static void place(HWND h, int x, int y, int w, int hgt, bool vis) {
    MoveWindow(h, x, y, w, hgt, TRUE);
    ShowWindow(h, vis ? SW_SHOW : SW_HIDE);
}

static void layoutPreview() {
    RECT rc;
    GetClientRect(g_hwnd, &rc);
    const int PX = 462;
    int pw = rc.right - PX - 14;
    if (pw < 120) pw = 120;
    MoveWindow(g_lblPreviewTitle, PX, 12, pw, 24, TRUE);
    MoveWindow(g_preview, PX, 40, pw, rc.bottom - 56, TRUE);
}

// 日志窗口延伸到窗口下边界
static void layoutLog() {
    RECT rc;
    GetClientRect(g_hwnd, &rc);
    int logH = rc.bottom - g_genBtnBottom - 6;
    if (logH < 60) logH = 60;
    MoveWindow(g_stStatus, 24, g_genBtnBottom, 416, logH, TRUE);
}

static void relayout() {
    GearType t = g_gearType;
    int y = 36;
    const int LX = 20, LW = 150, EX = 176, EW = 210;
    bool stacked = (t == GT_STACKED);
    bool internal = (t == GT_INTERNAL);

    place(g_lblLang, LX, y + 3, LW, 18, true); place(g_cmbLang, EX, y, EW, 160, true); y += 26;
    place(g_lblGear, LX, y + 3, LW, 18, true); place(g_cmbGear, EX, y, EW, 160, true); y += 36;

    // ---- 常规参数（非多层模式才显示）----
    bool showNormal = !stacked;

    place(g_lblDia, LX, y + 3, LW, 18, showNormal); place(g_edDia, EX, y, EW, 22, showNormal); y += (showNormal ? 26 : 0);
    place(g_lblTeeth, LX, y + 3, LW, 18, showNormal); place(g_edTeeth, EX, y, EW, 22, showNormal); y += (showNormal ? 26 : 0);

    bool crownWheel = (t == GT_CROWNWHEEL);
    place(g_lblThick, LX, y + 3, LW, 18, showNormal && !crownWheel); place(g_edThick, EX, y, EW, 22, showNormal && !crownWheel);
    place(g_lblSheet, LX, y + 3, LW, 18, showNormal && crownWheel); place(g_edSheet, EX, y, EW, 22, showNormal && crownWheel);
    y += (showNormal ? 26 : 0);

    bool crowned = (t == GT_CROWNED);
    place(g_lblFillet, LX, y + 3, LW, 18, showNormal && crowned); place(g_edFillet, EX, y, EW, 22, showNormal && crowned);
    place(g_lblToothH, LX, y + 3, LW, 18, showNormal && crownWheel); place(g_edToothH, EX, y, EW, 22, showNormal && crownWheel);
    if (showNormal && (crowned || crownWheel)) y += 26;

    // 内齿轮无中心凸起/孔（中心本身已空）
    bool showBoss = showNormal && !internal;
    place(g_chkBoss, LX, y, 210, 22, showBoss); y += (showBoss ? 24 : 0);
    place(g_lblBossThick, LX + 18, y + 3, LW - 18, 18, showBoss); place(g_edBossThick, EX, y, EW, 22, showBoss); y += (showBoss ? 26 : 0);
    place(g_lblBossDia, LX + 18, y + 3, LW - 18, 18, showBoss); place(g_edBossDia, EX, y, EW, 22, showBoss); y += (showBoss ? 26 : 0);

    bool showHole = showNormal && !internal;
    place(g_chkHole, LX, y, 210, 22, showHole); y += (showHole ? 24 : 0);
    place(g_lblHoleDia, LX + 18, y + 3, LW - 18, 18, showHole); place(g_edHoleDia, EX, y, EW, 22, showHole); y += (showHole ? 26 : 0);
    place(g_chkShape, LX + 18, y, 210, 22, showHole); y += (showHole ? 24 : 0);
    place(g_lblShape, LX + 18, y + 3, LW - 18, 18, showHole); place(g_cmbShape, EX, y, EW, 160, showHole); y += (showHole ? 26 : 0);

    bool shapeChecked = showHole && isChecked(g_chkShape);
    place(g_lblParam, LX + 18, y + 3, LW - 18, 18, shapeChecked); place(g_edParam, EX, y, EW, 22, shapeChecked);
    place(g_lblLen, LX + 18, y + 3, 50, 18, shapeChecked); place(g_edLen, LX + 72, y, 50, 22, shapeChecked);
    place(g_lblWid, LX + 128, y + 3, 40, 18, shapeChecked); place(g_edWid, LX + 172, y, 50, 22, shapeChecked);
    place(g_chkLock, EX + 110, y - 1, 100, 22, shapeChecked);
    y += (shapeChecked ? 28 : 0);

    // ---- 倒角控件（普通齿轮和内齿轮可见）----
    bool showChamfer = showNormal && (t == GT_SPUR || t == GT_INTERNAL);
    place(g_chkChamfer, LX, y, 210, 22, showChamfer); y += (showChamfer ? 24 : 0);
    place(g_lblChamferSize, LX + 18, y + 3, LW - 18, 18, showChamfer); place(g_edChamferSize, EX, y, 80, 22, showChamfer);
    place(g_lblChamferLoc, LX + 110, y + 3, 70, 18, showChamfer); place(g_cmbChamferLoc, EX + 90, y, 120, 160, showChamfer);
    y += (showChamfer ? 26 : 0);

    // 倒角与输出设置之间的间距
    y += 22;

    // ---- 多层齿轮参数（仅多层模式显示）----
    if (stacked) {
        place(g_lblLayerCount, LX, y + 3, LW, 18, true);
        place(g_edLayerCount, EX, y, 70, 22, true);
        place(g_btnAddLayer, EX + 78, y, 80, 22, true);
        place(g_btnDelLayer, EX + 164, y, 80, 22, true);
        y += 30;
        place(g_lvLayers, LX, y, 366, 170, true);
        y += 180;
    } else {
        place(g_lblLayerCount, LX, y + 3, LW, 18, false);
        place(g_edLayerCount, EX, y, 70, 22, false);
        place(g_btnAddLayer, EX + 78, y, 80, 22, false);
        place(g_btnDelLayer, EX + 164, y, 80, 22, false);
        place(g_lvLayers, LX, y, 366, 170, false);
    }

    place(g_lblUnit, LX, y + 3, LW, 18, true); place(g_cmbUnit, EX, y, EW, 160, true); y += 28;
    place(g_lblExportFmt, LX, y + 3, LW, 18, true); place(g_cmbExportFmt, EX, y, EW, 160, true); y += 28;

    place(g_btnGen, LX + 18, y, EW, 36, true); y += 42;
    g_genBtnBottom = y;
    layoutLog();

    updateHoleControls();
    layoutPreview();
}

static void applyLanguage() {
    SetWindowTextW(g_hwnd, tr(S_WINDOW_TITLE));
    SetWindowTextW(g_lblLang, tr(S_LANG));
    SetWindowTextW(g_lblGear, tr(S_GEARTYPE));
    SetWindowTextW(g_lblDia, tr(S_DIA));
    SetWindowTextW(g_lblTeeth, tr(S_TEETH));
    SetWindowTextW(g_lblThick, tr(S_THICK));
    SetWindowTextW(g_lblSheet, tr(S_SHEET));
    SetWindowTextW(g_lblToothH, tr(S_TOOTHH));
    SetWindowTextW(g_lblFillet, tr(S_FILLET));
    SetWindowTextW(g_lblUnit, tr(S_UNIT));
    SetWindowTextW(g_lblExportFmt, tr(S_EXPORT_FMT));
    SetWindowTextW(g_chkBoss, tr(S_BOSS));
    SetWindowTextW(g_lblBossThick, tr(S_BOSS_THICK));
    SetWindowTextW(g_lblBossDia, tr(S_BOSS_DIA));
    SetWindowTextW(g_chkHole, tr(S_HOLE));
    SetWindowTextW(g_lblHoleDia, tr(S_HOLE_DIA));
    SetWindowTextW(g_chkShape, tr(S_HOLE_SHAPE_CHK));
    SetWindowTextW(g_lblShape, tr(S_HOLE_SHAPE));
    SetWindowTextW(g_lblLen, tr(S_LEN));
    SetWindowTextW(g_lblWid, tr(S_WID));
    SetWindowTextW(g_chkLock, tr(S_LOCK));
    SetWindowTextW(g_lblPreviewTitle, tr(S_PREVIEW_TITLE));
    SetWindowTextW(g_lblLayerCount, tr(S_LAYER_COUNT));
    SetWindowTextW(g_btnAddLayer, tr(S_LAYER_ADD));
    SetWindowTextW(g_btnDelLayer, tr(S_LAYER_DEL));
    // 倒角控件
    SetWindowTextW(g_chkChamfer, tr(S_CHAMFER));
    SetWindowTextW(g_lblChamferSize, tr(S_CHAMFER_SIZE));
    SetWindowTextW(g_lblChamferLoc, tr(S_CHAMFER_LOC));
    // 外径标签随齿轮类型切换
    SetWindowTextW(g_lblDia, (g_gearType == GT_INTERNAL) ? tr(S_DIA_INT) : tr(S_DIA));

    // 齿轮类型下拉
    {
        int sel = comboSel(g_cmbGear);
        SendMessageW(g_cmbGear, CB_RESETCONTENT, 0, 0);
        const wchar_t* items[] = { tr(S_TYPE_SPUR), tr(S_TYPE_CROWNED), tr(S_TYPE_CROWNWHEEL), tr(S_TYPE_INTERNAL), tr(S_TYPE_STACKED) };
        for (int i = 0; i < 5; ++i) SendMessageW(g_cmbGear, CB_ADDSTRING, 0, (LPARAM)items[i]);
        SendMessageW(g_cmbGear, CB_SETCURSEL, sel < 0 ? 0 : sel, 0);
    }
    // 语言下拉
    {
        int sel = comboSel(g_cmbLang);
        SendMessageW(g_cmbLang, CB_RESETCONTENT, 0, 0);
        SendMessageW(g_cmbLang, CB_ADDSTRING, 0, (LPARAM)tr(S_LANG_ZH));
        SendMessageW(g_cmbLang, CB_ADDSTRING, 0, (LPARAM)tr(S_LANG_EN));
        SendMessageW(g_cmbLang, CB_SETCURSEL, (int)g_lang, 0);
    }
    // 单位下拉
    {
        int sel = comboSel(g_cmbUnit);
        SendMessageW(g_cmbUnit, CB_RESETCONTENT, 0, 0);
        const wchar_t* items[] = { tr(S_UNIT_MM), tr(S_UNIT_CM), tr(S_UNIT_INCH), tr(S_UNIT_M) };
        for (int i = 0; i < 4; ++i) SendMessageW(g_cmbUnit, CB_ADDSTRING, 0, (LPARAM)items[i]);
        SendMessageW(g_cmbUnit, CB_SETCURSEL, sel < 0 ? 3 : sel, 0);
    }
    // 导出格式下拉
    {
        int sel = comboSel(g_cmbExportFmt);
        SendMessageW(g_cmbExportFmt, CB_RESETCONTENT, 0, 0);
        const wchar_t* items[] = { tr(S_FMT_STL), tr(S_FMT_STEP) };
        for (int i = 0; i < 2; ++i) SendMessageW(g_cmbExportFmt, CB_ADDSTRING, 0, (LPARAM)items[i]);
        SendMessageW(g_cmbExportFmt, CB_SETCURSEL, sel < 0 ? 0 : sel, 0);
    }
    // 孔形状下拉
    {
        int sel = comboSel(g_cmbShape);
        SendMessageW(g_cmbShape, CB_RESETCONTENT, 0, 0);
        const wchar_t* items[] = { tr(S_SHAPE_D), tr(S_SHAPE_SQ), tr(S_SHAPE_POLY), tr(S_SHAPE_STAR) };
        for (int i = 0; i < 4; ++i) SendMessageW(g_cmbShape, CB_ADDSTRING, 0, (LPARAM)items[i]);
        SendMessageW(g_cmbShape, CB_SETCURSEL, sel < 0 ? 0 : sel, 0);
    }
    // 倒角位置下拉
    {
        int sel = comboSel(g_cmbChamferLoc);
        SendMessageW(g_cmbChamferLoc, CB_RESETCONTENT, 0, 0);
        const wchar_t* items[] = { tr(S_CHAMFER_NONE), tr(S_CHAMFER_TOP), tr(S_CHAMFER_BOTTOM), tr(S_CHAMFER_BOTH), tr(S_CHAMFER_TIP) };
        for (int i = 0; i < 5; ++i) SendMessageW(g_cmbChamferLoc, CB_ADDSTRING, 0, (LPARAM)items[i]);
        SendMessageW(g_cmbChamferLoc, CB_SETCURSEL, sel < 0 ? 0 : sel, 0);
    }
    updateHoleControls();
    updateGenButtonText();
    updateLayerListColumns();
    refreshLayerList();
}

// ---------------- 读取 / 校验 / 预览 ----------------
static GearParams readParamsFromUI() {
    GearParams p;
    p.outerDiameter  = readDouble(g_edDia);
    p.teeth          = (int)readLong(g_edTeeth);
    p.thickness      = readDouble(g_edThick);
    p.filletRadius   = readDouble(g_edFillet);
    p.sheetThickness = readDouble(g_edSheet);
    p.toothHeight    = readDouble(g_edToothH);
    p.hasBoss        = isChecked(g_chkBoss);
    p.bossThickness  = readDouble(g_edBossThick);
    p.bossDiameter   = readDouble(g_edBossDia);
    p.hasHole        = isChecked(g_chkHole);
    p.holeDiameter   = readDouble(g_edHoleDia);
    p.holeShape      = HS_CIRCLE;
    p.dLength        = 0.0;
    p.dWidth         = 0.0;
    p.holeSides      = 0;
    if (p.hasHole && isChecked(g_chkShape)) {
        int sel = comboSel(g_cmbShape);
        switch (sel) {
        case 0: p.holeShape = HS_D;       p.dLength = readDouble(g_edLen); p.dWidth = readDouble(g_edWid); break;
        case 1: p.holeShape = HS_SQUARE;  break;
        case 2: p.holeShape = HS_POLYGON; p.holeSides = (int)readLong(g_edParam); break;
        case 3: p.holeShape = HS_STAR;    p.holeSides = (int)readLong(g_edParam); break;
        }
    }
    // 倒角
    p.hasChamfer   = isChecked(g_chkChamfer);
    p.chamferSize  = readDouble(g_edChamferSize);
    p.chamferLoc   = comboSel(g_cmbChamferLoc);
    if (p.chamferLoc < 0) p.chamferLoc = CL_NONE;
    // 内齿轮无中心凸起/孔
    if (g_gearType == GT_INTERNAL) {
        p.hasBoss = false;
        p.hasHole = false;
    }
    return p;
}

static bool validateParams(HWND hwnd, const GearParams& p, GearType t, bool show) {
    auto err = [&](int k) -> bool { if (show) msgBox(hwnd, tr(k)); return false; };

    if (p.outerDiameter <= 0) return err(S_E_DIA);
    if (p.teeth < 5)          return err(S_E_TEETH);

    double m = (t == GT_INTERNAL) ? p.outerDiameter / (p.teeth + 6.5) : p.outerDiameter / (p.teeth + 2.0);
    double rootDia = m * (p.teeth - 2.5);

    if (t == GT_CROWNWHEEL) {
        if (p.sheetThickness <= 0) return err(S_E_SHEET);
        if (p.toothHeight <= 0)    return err(S_E_TOOTHH);
    } else {
        if (p.thickness <= 0) return err(S_E_THICK);
        if (t == GT_CROWNED) {
            if (p.filletRadius <= 0) return err(S_E_FILLET);
            if (p.filletRadius >= 1.1 * m) return err(S_E_FILLET_BIG);
        }
    }

    // 倒角校验
    if (p.hasChamfer && p.chamferSize <= 0) return err(S_E_CHAMFER);

    if (p.hasBoss) {
        if (p.bossThickness <= 0) return err(S_E_BOSS_THICK);
        if (p.bossDiameter <= 0)  return err(S_E_BOSS_DIA);
        double bossLimitDia = (t == GT_CROWNWHEEL) ? p.outerDiameter * 0.62 : rootDia;
        if (p.bossDiameter >= bossLimitDia) {
            if (show) { wchar_t b[256]; swprintf(b, 256, tr(S_E_BOSS_BIG), p.bossDiameter, bossLimitDia); msgBox(hwnd, b); }
            return false;
        }
    }

    if (p.hasHole) {
        double holeMaxDia;
        switch (p.holeShape) {
        case HS_CIRCLE:  holeMaxDia = p.holeDiameter; break;
        case HS_SQUARE:  holeMaxDia = p.holeDiameter * sqrt(2.0); break;
        case HS_D:       holeMaxDia = p.dLength; break;
        case HS_POLYGON: holeMaxDia = p.holeDiameter; break;
        default:         holeMaxDia = p.holeDiameter; break;
        }
        if (p.holeShape != HS_D && p.holeDiameter <= 0) return err(S_E_HOLE_DIA);
        if (p.holeShape == HS_D) {
            if (p.dLength <= 0 || p.dWidth <= 0) return err(S_E_HOLE_D);
            if (p.dWidth >= p.dLength || p.dWidth <= p.dLength * 0.5) return err(S_E_HOLE_D_RATIO);
        }
        if ((p.holeShape == HS_POLYGON || p.holeShape == HS_STAR) && p.holeSides < 3) return err(S_E_HOLE_SIDES);

        double limitDia;
        const wchar_t* word;
        if (t == GT_CROWNWHEEL) { limitDia = p.outerDiameter * 0.62; word = tr(S_E_ROOT_WORD); }
        else if (p.hasBoss)     { limitDia = p.bossDiameter; word = tr(S_E_BOSS_WORD); }
        else                    { limitDia = rootDia; word = tr(S_E_ROOT_WORD); }

        if (holeMaxDia >= limitDia) {
            if (show) { wchar_t b[256]; swprintf(b, 256, tr(S_E_HOLE_BIG), holeMaxDia, word, limitDia); msgBox(hwnd, b); }
            return false;
        }
    }
    return true;
}

static void rebuildPreview() {
    g_previewMesh.tris.clear();
    if (g_gearType == GT_STACKED) {
        if (validateLayerParams(g_hwnd, false))
            g_previewMesh = buildStackedMesh(g_layers);
    } else {
        GearParams p = readParamsFromUI();
        if (validateParams(g_hwnd, p, g_gearType, false))
            g_previewMesh = buildMesh(p, g_gearType);
    }
    if (g_preview) InvalidateRect(g_preview, NULL, FALSE);
}

// ---------------- 预览渲染 ----------------
static COLORREF shadeColor(int q) {
    int r = 182, g = 188, b = 198;
    return RGB(r * q / 32, g * q / 32, b * q / 32);
}
static HBRUSH shadeBrush(int q) {
    static HBRUSH cache[33] = { 0 };
    if (!cache[q]) cache[q] = CreateSolidBrush(shadeColor(q));
    return cache[q];
}

// 纯软件 3D 渲染：yaw/pitch 旋转 → 投影到 2D → 背面剔除 → 光照着色 → 画家算法排序
static void renderMeshPreview(HDC hdc, RECT rc, const Mesh& m) {
    if (m.tris.empty()) {
        SetTextColor(hdc, RGB(140, 150, 160));
        SetBkMode(hdc, TRANSPARENT);
        HFONT f = makeFont(13, false);
        HGDIOBJ of = SelectObject(hdc, f);
        DrawTextW(hdc, tr(S_PREVIEW_PLACEHOLDER), -1, &rc, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
        SelectObject(hdc, of);
        DeleteObject(f);
        return;
    }

    double mnx = 1e30, mny = 1e30, mnz = 1e30, mxx = -1e30, mxy = -1e30, mxz = -1e30;
    for (auto& t : m.tris) {
        for (int k = 0; k < 3; ++k) {
            double x = t.v[k].x, y = t.v[k].y, z = t.v[k].z;
            if (x < mnx) mnx = x; if (x > mxx) mxx = x;
            if (y < mny) mny = y; if (y > mxy) mxy = y;
            if (z < mnz) mnz = z; if (z > mxz) mxz = z;
        }
    }
    double cx = (mnx + mxx) * 0.5, cy = (mny + mxy) * 0.5, cz = (mnz + mxz) * 0.5;
    double span = max(max(mxx - mnx, mxy - mny), mxz - mnz);
    if (span < 1e-9) return;

    int W = rc.right - rc.left, H = rc.bottom - rc.top;
    double sc = min(W, H) * 0.42 / (span * 0.5) * g_zoom;
    int ox = rc.left + W / 2, oy = rc.top + H / 2;

    double A = g_yaw, B = g_pitch;
    double ca = cos(A), sa = sin(A), cb = cos(B), sb = sin(B);
    double lx = 0.45, ly = -0.55, lz = 0.7;
    double ll = sqrt(lx * lx + ly * ly + lz * lz);
    lx /= ll; ly /= ll; lz /= ll;

    struct P { POINT p[3]; int zint; int q; };
    vector<P> polys;
    polys.reserve(m.tris.size());

    for (auto& t : m.tris) {
        POINT s[3];
        double zs[3];
        double n1x = t.n.x * ca - t.n.y * sa;
        double n1y = t.n.x * sa + t.n.y * ca;
        double ncx = n1x;
        double ncy = n1y * cb - t.n.z * sb;
        double ncz = n1y * sb + t.n.z * cb;

        for (int k = 0; k < 3; ++k) {
            double x = t.v[k].x - cx, y = t.v[k].y - cy, z = t.v[k].z - cz;
            double x1 = x * ca - y * sa;
            double y1 = x * sa + y * ca;
            double y2 = y1 * cb - z * sb;
            double z2 = y1 * sb + z * cb;
            s[k].x = (LONG)(ox + x1 * sc);
            s[k].y = (LONG)(oy - z2 * sc);
            zs[k] = y2;
        }
        double area = (double)(s[1].x - s[0].x) * (s[2].y - s[0].y) - (double)(s[1].y - s[0].y) * (s[2].x - s[0].x);
        if (fabs(area) < 0.5) continue;

        double b = fabs(ncx * lx + ncy * ly + ncz * lz);
        double f = 0.5 + 0.5 * b;
        int q = (int)(f * 32 + 0.5);
        if (q > 32) q = 32; if (q < 0) q = 0;

        P pp;
        pp.p[0] = s[0]; pp.p[1] = s[1]; pp.p[2] = s[2];
        pp.zint = (int)((zs[0] + zs[1] + zs[2]) * 1000.0);
        pp.q = q;
        polys.push_back(pp);
    }

    sort(polys.begin(), polys.end(), [](const P& a, const P& b) { return a.zint > b.zint; });

    HGDIOBJ oldPen = SelectObject(hdc, GetStockObject(DC_PEN));
    for (auto& pp : polys) {
        HBRUSH br = shadeBrush(pp.q);
        HGDIOBJ oldBr = SelectObject(hdc, br);
        SetDCPenColor(hdc, shadeColor(pp.q));
        Polygon(hdc, pp.p, 3);
        SelectObject(hdc, oldBr);
    }
    SelectObject(hdc, oldPen);
}

// ---------------- 生成导出流程 ----------------
// 全程输出日志：读取参数 → 校验 → 构建网格 → 准备导出 → 选文件 → 写入
static void onGenerate(HWND hwnd) {
    logMsg(L"========== Generation Started ==========");

    const wchar_t* typeNames[] = { L"Spur", L"Crowned", L"CrownWheel", L"Internal", L"Stacked" };

    // [1/6] 读取参数
    logMsg(L"[1/6] Reading parameters...");
    GearParams p;
    bool isStacked = (g_gearType == GT_STACKED);
    if (!isStacked) {
        p = readParamsFromUI();
        logMsg(L"     Type=%ls  Teeth=%d  OuterDia=%.3fmm  Thickness=%.3fmm",
               typeNames[(int)g_gearType], p.teeth, p.outerDiameter, p.thickness);
    } else {
        logMsg(L"     Type=%ls  Layers=%d", typeNames[(int)g_gearType], (int)g_layers.size());
    }

    // [2/6] 校验
    logMsg(L"[2/6] Validating parameters...");
    if (isStacked) {
        if (!validateLayerParams(hwnd, true)) {
            logErr(L"Validation failed -- check your inputs");
            logMsg(L"========== Generation Aborted ==========");
            return;
        }
    } else {
        if (!validateParams(hwnd, p, g_gearType, true)) {
            logErr(L"Validation failed -- check your inputs");
            logMsg(L"========== Generation Aborted ==========");
            return;
        }
    }
    logMsg(L"     OK -- all parameters valid");

    // [3/6] 构建网格
    logMsg(L"[3/6] Building mesh...");
    Mesh mesh = isStacked ? buildStackedMesh(g_layers) : buildMesh(p, g_gearType);
    logMsg(L"     Mesh built: %d triangles", (int)mesh.tris.size());

    // [4/6] 准备导出
    logMsg(L"[4/6] Preparing export settings...");
    int fmtSel = comboSel(g_cmbExportFmt);
    if (fmtSel < 0) fmtSel = 0;
    bool isStep = (fmtSel == 1);
    int unitSel = comboSel(g_cmbUnit);
    if (unitSel < 0) unitSel = 0;
    double scale = unitScale(unitSel);
    const wchar_t* unitNames[] = { L"mm", L"cm", L"inch", L"m" };
    logMsg(L"     Format=%ls  Unit=%ls  Scale=%.6f",
           isStep ? L"STEP" : L"STL", unitNames[unitSel], scale);

    // [5/6] 选择输出文件
    logMsg(L"[5/6] Selecting output file...");
    OPENFILENAMEW ofn;
    ZeroMemory(&ofn, sizeof(ofn));
    wchar_t fileBuf[512];
    wcscpy(fileBuf, isStep ? tr(S_DEFAULT_FILE_STEP) : tr(S_DEFAULT_FILE));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = hwnd;
    wchar_t filter[256];
    if (isStep)
        swprintf(filter, 256, L"%ls%c*.step%c%ls%c*.*%c%c", tr(S_FILTER_STEP), 0, 0, tr(S_FILTER_ALL), 0, 0, 0);
    else
        swprintf(filter, 256, L"%ls%c*.stl%c%ls%c*.*%c%c", tr(S_FILTER_STL), 0, 0, tr(S_FILTER_ALL), 0, 0, 0);
    ofn.lpstrFilter = filter;
    ofn.lpstrFile = fileBuf;
    ofn.nMaxFile = 512;
    ofn.lpstrDefExt = isStep ? L"step" : L"stl";
    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST;
    if (!GetSaveFileNameW(&ofn)) {
        logMsg(L"     Cancelled by user");
        logMsg(L"========== Generation Aborted ==========");
        return;
    }
    logMsg(L"     Target: %ls", ofn.lpstrFile);

    // [6/6] 写入文件
    logMsg(L"[6/6] Writing file...");
    bool ok = isStep ? writeSTEP(ofn.lpstrFile, mesh, scale) : writeBinarySTL(ofn.lpstrFile, mesh, scale);
    if (ok) {
        if (isStacked) {
            logMsg(L"     DONE!  Layers=%d  Triangles=%d  Unit=%ls",
                   (int)g_layers.size(), (int)mesh.tris.size(), unitNames[unitSel]);
        } else {
            double m = (g_gearType == GT_INTERNAL) ? p.outerDiameter / (p.teeth + 6.5) : p.outerDiameter / (p.teeth + 2.0);
            logMsg(L"     DONE!  Triangles=%d  Module=%.3fmm  Unit=%ls",
                   (int)mesh.tris.size(), m, unitNames[unitSel]);
        }
        logMsg(L"========== Generation Complete ==========");
    } else {
        logErr(L"File write failed: %ls", ofn.lpstrFile);
        msgBox(hwnd, tr(S_WRITE_FAIL));
        logMsg(L"========== Generation Failed ==========");
    }
}

// 预览窗口过程：双缓冲绘制 + 左键拖拽旋转 + 滚轮缩放
static LRESULT CALLBACK PreviewProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_PAINT: {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hwnd, &ps);
        RECT rc;
        GetClientRect(hwnd, &rc);
        HDC mem = CreateCompatibleDC(hdc);
        HBITMAP bmp = CreateCompatibleBitmap(hdc, rc.right, rc.bottom);
        HGDIOBJ old = SelectObject(mem, bmp);

        HBRUSH bg = CreateSolidBrush(RGB(246, 248, 251));
        FillRect(mem, &rc, bg);
        DeleteObject(bg);

        renderMeshPreview(mem, rc, g_previewMesh);

        // 边框
        HBRUSH frame = (HBRUSH)GetStockObject(NULL_BRUSH);
        HGDIOBJ ob = SelectObject(mem, frame);
        HPEN pen = CreatePen(PS_SOLID, 1, RGB(210, 216, 224));
        HGDIOBJ op = SelectObject(mem, pen);
        Rectangle(mem, 0, 0, rc.right, rc.bottom);
        SelectObject(mem, op); DeleteObject(pen);
        SelectObject(mem, ob);

        BitBlt(hdc, 0, 0, rc.right, rc.bottom, mem, 0, 0, SRCCOPY);
        SelectObject(mem, old);
        DeleteObject(bmp);
        DeleteDC(mem);
        EndPaint(hwnd, &ps);
        return 0;
    }
    case WM_LBUTTONDOWN:  // 开始拖拽
        SetCapture(hwnd);
        g_dragLast.x = (int)(short)LOWORD(lp);
        g_dragLast.y = (int)(short)HIWORD(lp);
        return 0;
    case WM_MOUSEMOVE:   // 拖拽中：更新 yaw/pitch 旋转角
        if (GetCapture() == hwnd) {
            int x = (int)(short)LOWORD(lp);
            int y = (int)(short)HIWORD(lp);
            g_yaw   += (double)(x - g_dragLast.x) * 0.01;
            g_pitch += (double)(y - g_dragLast.y) * 0.01;
            if (g_pitch >  1.45) g_pitch =  1.45;
            if (g_pitch < -1.45) g_pitch = -1.45;
            g_dragLast.x = x;
            g_dragLast.y = y;
            InvalidateRect(hwnd, NULL, FALSE);
        }
        return 0;
    case WM_LBUTTONUP:
        ReleaseCapture();
        return 0;
    case WM_MOUSEWHEEL: {  // 滚轮缩放：上滚放大、下滚缩小
        int delta = GET_WHEEL_DELTA_WPARAM(wp);
        double factor = delta > 0 ? 1.15 : 1.0 / 1.15;
        g_zoom *= factor;
        if (g_zoom < 0.1) g_zoom = 0.1;
        if (g_zoom > 20.0) g_zoom = 20.0;
        InvalidateRect(hwnd, NULL, FALSE);
        return 0;
    }
    case WM_ERASEBKGND:
        return 1;
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

static void onCreate(HWND hwnd) {
    g_font = makeFont(13, false);

    // 标题
    HWND t = CreateWindowW(L"STATIC", L"", WS_CHILD | WS_VISIBLE, 24, 12, 430, 30, hwnd, nullptr, g_hInst, nullptr);
    HFONT fTitle = makeFont(19, true);
    SendMessageW(t, WM_SETFONT, (WPARAM)fTitle, TRUE);
    SetWindowTextW(t, tr(S_TITLE));

    // 语言 / 类型（位置由 relayout 统一管理，此处先建控件）
    g_lblLang = makeLabel(hwnd, L"", 24, 14, 150, 20);
    g_cmbLang = makeCombo(hwnd, IDC_LANG, 188, 14, 216, 160);
    g_lblGear = makeLabel(hwnd, L"", 24, 14, 150, 20);
    g_cmbGear = makeCombo(hwnd, IDC_GEARTYPE, 188, 14, 216, 160);

    g_lblDia = makeLabel(hwnd, L"", 24, 14, 150, 22);
    g_edDia = makeEdit(hwnd, IDC_DIA, 188, 14, 216, 26);
    SetWindowTextW(g_edDia, L"40");
    g_lblTeeth = makeLabel(hwnd, L"", 24, 14, 150, 22);
    g_edTeeth = makeEdit(hwnd, IDC_TEETH, 188, 14, 216, 26);
    SetWindowTextW(g_edTeeth, L"20");

    g_lblThick = makeLabel(hwnd, L"", 24, 14, 150, 22);
    g_edThick = makeEdit(hwnd, IDC_THICK, 188, 14, 216, 26);
    SetWindowTextW(g_edThick, L"5");
    g_lblSheet = makeLabel(hwnd, L"", 24, 14, 150, 22);
    g_edSheet = makeEdit(hwnd, IDC_SHEET, 188, 14, 216, 26);
    SetWindowTextW(g_edSheet, L"3");
    g_lblFillet = makeLabel(hwnd, L"", 24, 14, 150, 22);
    g_edFillet = makeEdit(hwnd, IDC_FILLET, 188, 14, 216, 26);
    SetWindowTextW(g_edFillet, L"1");
    g_lblToothH = makeLabel(hwnd, L"", 24, 14, 150, 22);
    g_edToothH = makeEdit(hwnd, IDC_TOOTH_HEIGHT, 188, 14, 216, 26);
    SetWindowTextW(g_edToothH, L"8");

    g_lblUnit = makeLabel(hwnd, L"", 24, 14, 150, 22);
    g_cmbUnit = makeCombo(hwnd, IDC_UNIT, 188, 14, 216, 160);

    g_lblExportFmt = makeLabel(hwnd, L"", 24, 14, 150, 22);
    g_cmbExportFmt = makeCombo(hwnd, IDC_EXPORT_FMT, 188, 14, 216, 160);

    g_chkBoss = makeCheck(hwnd, IDC_BOSS_CHK, L"", 24, 14, 240, 24);
    g_lblBossThick = makeLabel(hwnd, L"", 44, 14, 130, 22);
    g_edBossThick = makeEdit(hwnd, IDC_BOSS_THICK, 188, 14, 216, 26);
    SetWindowTextW(g_edBossThick, L"5");
    g_lblBossDia = makeLabel(hwnd, L"", 44, 14, 130, 22);
    g_edBossDia = makeEdit(hwnd, IDC_BOSS_DIA, 188, 14, 216, 26);
    SetWindowTextW(g_edBossDia, L"20");

    g_chkHole = makeCheck(hwnd, IDC_HOLE_CHK, L"", 24, 14, 240, 24);
    g_lblHoleDia = makeLabel(hwnd, L"", 44, 14, 130, 22);
    g_edHoleDia = makeEdit(hwnd, IDC_HOLE_DIA, 188, 14, 216, 26);
    SetWindowTextW(g_edHoleDia, L"8");

    g_chkShape = makeCheck(hwnd, IDC_HOLE_SHAPE_CHK, L"", 44, 14, 240, 24);
    g_lblShape = makeLabel(hwnd, L"", 44, 14, 130, 22);
    g_cmbShape = makeCombo(hwnd, IDC_HOLE_SHAPE_COMBO, 188, 14, 216, 160);
    g_lblParam = makeLabel(hwnd, L"", 44, 14, 130, 22);
    g_edParam = makeEdit(hwnd, IDC_HOLE_PARAM, 188, 14, 216, 26);
    SetWindowTextW(g_edParam, L"6");

    g_lblLen = makeLabel(hwnd, L"", 44, 14, 30, 22);
    g_edLen = makeEdit(hwnd, IDC_HOLE_LEN, 76, 14, 56, 26);
    SetWindowTextW(g_edLen, L"10");
    g_lblWid = makeLabel(hwnd, L"", 142, 14, 30, 22);
    g_edWid = makeEdit(hwnd, IDC_HOLE_WID, 174, 14, 56, 26);
    SetWindowTextW(g_edWid, L"8");
    g_chkLock = makeCheck(hwnd, IDC_HOLE_LOCK, L"", 236, 14, 118, 26);

    // 倒角控件
    g_chkChamfer = makeCheck(hwnd, IDC_CHAMFER_CHK, L"", 24, 14, 240, 24);
    g_lblChamferSize = makeLabel(hwnd, L"", 44, 14, 130, 22);
    g_edChamferSize = makeEdit(hwnd, IDC_CHAMFER_SIZE, 188, 14, 80, 26);
    SetWindowTextW(g_edChamferSize, L"0.5");
    g_lblChamferLoc = makeLabel(hwnd, L"", 286, 14, 70, 22);
    g_cmbChamferLoc = makeCombo(hwnd, IDC_CHAMFER_LOC, 366, 14, 120, 160);

    // 多层齿轮控件
    g_lblLayerCount = makeLabel(hwnd, L"", 24, 14, 170, 22);
    g_edLayerCount = makeEdit(hwnd, IDC_LAYER_COUNT, 200, 14, 80, 26);
    SetWindowTextW(g_edLayerCount, L"2");
    g_btnAddLayer = CreateWindowW(L"BUTTON", L"", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
                                  290, 14, 90, 26, hwnd, (HMENU)(INT_PTR)IDC_LAYER_ADD, g_hInst, nullptr);
    SendMessageW(g_btnAddLayer, WM_SETFONT, (WPARAM)g_font, TRUE);
    g_btnDelLayer = CreateWindowW(L"BUTTON", L"", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
                                  388, 14, 90, 26, hwnd, (HMENU)(INT_PTR)IDC_LAYER_DEL, g_hInst, nullptr);
    SendMessageW(g_btnDelLayer, WM_SETFONT, (WPARAM)g_font, TRUE);
    g_lvLayers = CreateWindowExW(0, WC_LISTVIEWW, L"",
                                 WS_CHILD | WS_BORDER | LVS_REPORT | LVS_SINGLESEL | LVS_SHOWSELALWAYS,
                                 24, 14, 388, 200, hwnd, (HMENU)(INT_PTR)IDC_LAYER_LIST, g_hInst, nullptr);
    SendMessageW(g_lvLayers, WM_SETFONT, (WPARAM)g_font, TRUE);
    ListView_SetExtendedListViewStyle(g_lvLayers, LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);

    g_btnGen = CreateWindowW(L"BUTTON", L"", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_DEFPUSHBUTTON,
                             54, 14, 220, 40, hwnd, (HMENU)(INT_PTR)IDC_GEN, g_hInst, nullptr);
    HFONT fBtn = makeFont(14, true);
    SendMessageW(g_btnGen, WM_SETFONT, (WPARAM)fBtn, TRUE);

    // 日志窗口：使用 RichEdit 控件以支持逐行着色（错误红色、普通浅绿）
    g_stStatus = CreateWindowExW(WS_EX_CLIENTEDGE, g_richClass, L"",
                               WS_CHILD | WS_VISIBLE | WS_VSCROLL | ES_MULTILINE | ES_READONLY | ES_AUTOVSCROLL,
                               24, 14, 400, 64, hwnd, (HMENU)(INT_PTR)IDC_STATUS, g_hInst, nullptr);
    SendMessageW(g_stStatus, EM_SETBKGNDCOLOR, 0, (LPARAM)RGB(0, 0, 0));
    // 默认文字浅绿、等宽字体（终端风格）
    CHARFORMAT2 dcf;
    ZeroMemory(&dcf, sizeof(dcf));
    dcf.cbSize = sizeof(dcf);
    dcf.dwMask = CFM_COLOR;
    dcf.crTextColor = RGB(180, 230, 180);
    SendMessageW(g_stStatus, EM_SETCHARFORMAT, SCF_DEFAULT, (LPARAM)&dcf);
    HFONT fLog = CreateFontW(-14, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                             DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
                             DEFAULT_PITCH | FF_DONTCARE, L"Consolas");
    SendMessageW(g_stStatus, WM_SETFONT, (WPARAM)fLog, TRUE);

    // 预览标题 + 预览窗口
    g_lblPreviewTitle = CreateWindowW(L"STATIC", L"", WS_CHILD | WS_VISIBLE, 478, 12, 400, 24, hwnd, nullptr, g_hInst, nullptr);
    HFONT fPrev = makeFont(16, true);
    SendMessageW(g_lblPreviewTitle, WM_SETFONT, (WPARAM)fPrev, TRUE);

    g_preview = CreateWindowW(L"GearPreviewWnd", L"", WS_CHILD | WS_VISIBLE,
                              478, 40, 400, 400, hwnd, (HMENU)(INT_PTR)IDC_PREVIEW, g_hInst, nullptr);

    // 初始状态
    EnableWindow(g_edBossThick, FALSE);
    EnableWindow(g_edBossDia, FALSE);
    EnableWindow(g_edChamferSize, FALSE);
    EnableWindow(g_cmbChamferLoc, FALSE);

    initDefaultLayers();
    applyLanguage();
    relayout();
    rebuildPreview();
    logMsg(L"Simple Gear Generator initialized.");
}

static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_CREATE:
        onCreate(hwnd);
        return 0;
    case WM_SIZE:
        layoutPreview();
        layoutLog();
        return 0;
    case WM_COMMAND: {
        int id = LOWORD(wp);
        int code = HIWORD(wp);

        if (id == IDC_GEN && code == BN_CLICKED) {
            onGenerate(hwnd);
        }
        else if (id == IDC_GEARTYPE && code == CBN_SELCHANGE) {
            g_gearType = (GearType)comboSel(g_cmbGear);
            const wchar_t* typeNames[] = { L"Spur", L"Crowned", L"CrownWheel", L"Internal", L"Stacked" };
            logMsg(L"Gear type changed to %ls", typeNames[(int)g_gearType]);
            // 切换齿轮类型时更新外径标签
            SetWindowTextW(g_lblDia, (g_gearType == GT_INTERNAL) ? tr(S_DIA_INT) : tr(S_DIA));
            relayout();
            rebuildPreview();
        }
        else if (id == IDC_LANG && code == CBN_SELCHANGE) {
            g_lang = (comboSel(g_cmbLang) == 1) ? LANG_EN : LANG_ZH;
            applyLanguage();
        }
        else if (id == IDC_EXPORT_FMT && code == CBN_SELCHANGE) {
            updateGenButtonText();
            logMsg(L"Export format -> %ls", comboSel(g_cmbExportFmt) == 1 ? L"STEP" : L"STL");
        }
        else if (id == IDC_UNIT && code == CBN_SELCHANGE) {
            const wchar_t* unitNames[] = { L"mm", L"cm", L"inch", L"m" };
            int sel = comboSel(g_cmbUnit);
            if (sel >= 0 && sel < 4) logMsg(L"Output unit -> %ls", unitNames[sel]);
        }
        else if (id == IDC_BOSS_CHK && code == BN_CLICKED) {
            BOOL on = isChecked(g_chkBoss);
            EnableWindow(g_edBossThick, on);
            EnableWindow(g_edBossDia, on);
            rebuildPreview();
        }
        else if (id == IDC_HOLE_CHK && code == BN_CLICKED) {
            updateHoleControls();
            rebuildPreview();
        }
        else if (id == IDC_HOLE_SHAPE_CHK && code == BN_CLICKED) {
            updateHoleControls();
            relayout();
            rebuildPreview();
        }
        else if (id == IDC_HOLE_SHAPE_COMBO && code == CBN_SELCHANGE) {
            int sel = comboSel(g_cmbShape);
            if (sel == 2) SetWindowTextW(g_edParam, L"6");
            else if (sel == 3) SetWindowTextW(g_edParam, L"5");
            updateHoleControls();
            rebuildPreview();
        }
        else if (id == IDC_HOLE_LOCK && code == BN_CLICKED) {
            if (isChecked(g_chkLock)) {
                double L = readDouble(g_edLen);
                double W = readDouble(g_edWid);
                g_lockRatio = (L > 0) ? W / L : 1.0;
            }
        }
        else if (id == IDC_CHAMFER_CHK && code == BN_CLICKED) {
            BOOL on = isChecked(g_chkChamfer);
            EnableWindow(g_edChamferSize, on);
            EnableWindow(g_cmbChamferLoc, on);
            rebuildPreview();
        }
        else if (id == IDC_CHAMFER_LOC && code == CBN_SELCHANGE) {
            rebuildPreview();
        }
        else if (id == IDC_HOLE_LEN && code == EN_CHANGE) {
            syncLenWid(true);
            rebuildPreview();
        }
        else if (id == IDC_HOLE_WID && code == EN_CHANGE) {
            syncLenWid(false);
            rebuildPreview();
        }
        else if (id == IDC_LAYER_ADD && code == BN_CLICKED) {
            GearParams np = {};
            np.outerDiameter = 40; np.teeth = 20; np.thickness = 5;
            g_layers.push_back({ GT_SPUR, np });
            refreshLayerList();
            rebuildPreview();
            logMsg(L"Layer added: total=%d", (int)g_layers.size());
        }
        else if (id == IDC_LAYER_DEL && code == BN_CLICKED) {
            int sel = ListView_GetSelectionMark(g_lvLayers);
            if (sel >= 0 && sel < (int)g_layers.size()) {
                g_layers.erase(g_layers.begin() + sel);
                refreshLayerList();
                rebuildPreview();
                logMsg(L"Layer removed: total=%d", (int)g_layers.size());
            }
        }
        else if (id == IDC_LAYER_COUNT && code == EN_CHANGE) {
            if (g_layerSyncing) return 0;
            int cnt = (int)readLong(g_edLayerCount);
            if (cnt < 1) cnt = 1;
            while ((int)g_layers.size() < cnt) {
                GearParams np = {};
                np.outerDiameter = 40; np.teeth = 20; np.thickness = 5;
                g_layers.push_back({ GT_SPUR, np });
            }
            while ((int)g_layers.size() > cnt)
                g_layers.pop_back();
            refreshLayerList();
            rebuildPreview();
        }
        else if (code == EN_CHANGE &&
                 (id == IDC_DIA || id == IDC_TEETH || id == IDC_THICK || id == IDC_FILLET ||
                  id == IDC_SHEET || id == IDC_TOOTH_HEIGHT || id == IDC_BOSS_THICK ||
                  id == IDC_BOSS_DIA || id == IDC_HOLE_DIA || id == IDC_HOLE_PARAM ||
                  id == IDC_CHAMFER_SIZE)) {
            rebuildPreview();
        }
        return 0;
    }
    case WM_NOTIFY: {
        NMHDR* pnmh = (NMHDR*)lp;
        if (pnmh && pnmh->idFrom == IDC_LAYER_LIST) {
            if (pnmh->code == NM_DBLCLK) {
                NMITEMACTIVATE* pnmia = (NMITEMACTIVATE*)pnmh;
                if (pnmia->iItem >= 0)
                    showLayerEditDialog(pnmia->iItem);
            }
        }
        return DefWindowProcW(hwnd, msg, wp, lp);
    }
    case WM_CTLCOLORSTATIC:
    case WM_CTLCOLOREDIT: {
        HWND hChild = (HWND)lp;
        if (hChild == g_stStatus) {
            HDC hdc = (HDC)wp;
            SetTextColor(hdc, RGB(200, 230, 200));
            SetBkColor(hdc, RGB(0, 0, 0));
            static HBRUSH logBrush = nullptr;
            if (!logBrush) logBrush = CreateSolidBrush(RGB(0, 0, 0));
            return (LRESULT)logBrush;
        }
        break;
    }
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

int WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR, int nCmdShow) {
    g_hInst = hInst;
    initRichEdit(); // 加载 RichEdit DLL（彩色日志所需）

    INITCOMMONCONTROLSEX icc;
    icc.dwSize = sizeof(icc);
    icc.dwICC = ICC_STANDARD_CLASSES | ICC_LISTVIEW_CLASSES;
    InitCommonControlsEx(&icc);

    const wchar_t* cls = L"SimpleGearGeneratorWnd";
    WNDCLASSW wc = {};
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInst;
    wc.lpszClassName = cls;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wc.hIcon = LoadIcon(hInst, MAKEINTRESOURCE(1));
    RegisterClassW(&wc);

    WNDCLASSW pc = {};
    pc.lpfnWndProc = PreviewProc;
    pc.hInstance = hInst;
    pc.lpszClassName = L"GearPreviewWnd";
    pc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    pc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    RegisterClassW(&pc);

    RECT r = { 0, 0, 1000, 680 };
    AdjustWindowRect(&r, WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX, FALSE);
    g_hwnd = CreateWindowW(cls, L"",
                           WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
                           CW_USEDEFAULT, CW_USEDEFAULT, r.right - r.left, r.bottom - r.top,
                           nullptr, nullptr, hInst, nullptr);
    SendMessage(g_hwnd, WM_SETICON, ICON_BIG, (LPARAM)LoadImage(hInst, MAKEINTRESOURCE(1), IMAGE_ICON, 256, 256, LR_DEFAULTCOLOR));
    SendMessage(g_hwnd, WM_SETICON, ICON_SMALL, (LPARAM)LoadImage(hInst, MAKEINTRESOURCE(1), IMAGE_ICON, 16, 16, LR_DEFAULTCOLOR));
    ShowWindow(g_hwnd, nCmdShow);
    UpdateWindow(g_hwnd);

    MSG msg;
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return (int)msg.wParam;
}
