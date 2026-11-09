// Offline checks of CartesianReferenceGenerator (no robot): step response (no overshoot, straight
// line, limits), 72 Hz noisy WebXR-like targets, continuous rotation through +-pi, lost packets.
//   pixi run test-reference   (args: [position_gain] [0 = held targets without twist] [noise scale])
#include <cstdio>
#include <cstdlib>
#include <random>
#include "control_mode/cartesian_target.hpp"

using V6 = Eigen::Matrix<double, 6, 1>;
double g_gain = 30.0;
CartesianFeedforwardConfig cfg()
{
    CartesianFeedforwardConfig c;
    c.reference_position_gain = g_gain;
    c.max_velocity = {0.7, 0.7, 0.7, 1.5, 1.5, 1.5};
    c.max_acceleration = {4, 4, 4, 8, 8, 8};
    c.max_jerk = {200, 200, 200, 400, 400, 400};
    return c;
}
double angle(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) { return rotationVector(a * b.inverse()).norm(); }

struct Stats { double vmax[2]{}, amax[2]{}, jmax[2]{}; };
void track(Stats& s, const V6& tw, const V6& acc, const V6& prev_acc, uint64_t steps)
{
    for (int p = 0; p < 2; p++)
        for (int i = 3 * p; i < 3 * p + 3; i++)
        {
            s.vmax[p] = std::max(s.vmax[p], std::abs(tw[i]));
            s.amax[p] = std::max(s.amax[p], std::abs(acc[i]));
            s.jmax[p] = std::max(s.jmax[p], std::abs(acc[i] - prev_acc[i]) / (0.001 * steps));
        }
}
int fails = 0;
void check(bool ok, const char* what) { std::printf("  [%s] %s\n", ok ? "ok" : "FAIL", what); fails += !ok; }

int main(int argc, char** argv)
{
    if (argc > 1) g_gain = std::atof(argv[1]);
    // 1) step: 10 cm along (1,1,0) + 90 deg about z, from rest
    {
        CartesianReferenceGenerator g;
        const Eigen::Vector3d p0(0.5, 0, 0.4);
        const Eigen::Quaterniond q0(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
        g.reset(p0, q0, cfg());
        const Eigen::Vector3d pt = p0 + Eigen::Vector3d(0.1, 0.1, 0) / std::sqrt(2.0) ;
        const Eigen::Quaterniond qt = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())) * q0;
        Eigen::Vector3d p; Eigen::Quaterniond q; V6 tw, acc, prev = V6::Zero(); Stats s;
        double overshoot = 0, offline = 0, t_settle = -1;
        const Eigen::Vector3d dir = (pt - p0).normalized();
        for (int k = 0; k < 3000; k++)
        {
            g.update(pt, qt, V6::Zero(), 1, p, q, tw, acc);
            track(s, tw, acc, prev, 1); prev = acc;
            const Eigen::Vector3d d = p - p0;
            overshoot = std::max(overshoot, d.dot(dir) - 0.1);
            offline = std::max(offline, (d - d.dot(dir) * dir).norm());
            if (t_settle < 0 && (p - pt).norm() < 1e-4 && angle(qt, q) < 1e-3) t_settle = k * 1e-3;
        }
        std::printf("step: settle %.3f s, overshoot %.2e m, off-line %.2e m, final %.2e m / %.2e rad\n", t_settle, overshoot, offline, (p - pt).norm(), angle(qt, q));
        std::printf("  lin v/a/j %.3f %.2f %.0f   rot v/a/j %.3f %.2f %.0f\n", s.vmax[0], s.amax[0], s.jmax[0], s.vmax[1], s.amax[1], s.jmax[1]);
        check(t_settle > 0 && t_settle < 1.5, "settles within 1.5 s (90 deg at 1.5 rad/s)");
        check(overshoot < 1e-4, "no overshoot (<0.1 mm)");
        check(offline < 1e-3, "straight line (<1 mm off)");
        check(s.vmax[0] <= 0.7 + 1e-9 && s.amax[0] <= 4 + 1e-6 && s.jmax[0] <= 200 * 1.01, "linear limits");
        check(s.vmax[1] <= 1.5 + 1e-9 && s.amax[1] <= 8 + 1e-6 && s.jmax[1] <= 400 * 1.01, "angular limits");
    }
    // 2) WebXR-like: smooth hand motion sampled at 72 Hz with noise, sample-and-hold, twist = 0
    {
        CartesianReferenceGenerator g;
        std::mt19937 rng(1); const double ns = argc > 3 ? std::atof(argv[3]) : 1.0; std::normal_distribution<double> n(0, 0.0005 * ns + 1e-12), nr(0, 0.003 * ns + 1e-12);
        const Eigen::Vector3d p0(0.5, 0, 0.4); const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
        g.reset(p0, q0, cfg());
        Eigen::Vector3d p, held = p0, prev_held = p0; Eigen::Quaterniond q, heldq = q0, prev_heldq = q0; V6 tw, acc, prev = V6::Zero(), est = V6::Zero(); Stats s; double t_s = 0;
        const bool estimate = !(argc > 2 && std::atoi(argv[2]) == 0);  // client twist (WebXR) by default
        double err = 0, rerr = 0, sumj = 0; int cnt = 0;
        for (int k = 0; k < 6000; k++)
        {
            const double t = k * 1e-3;
            auto hand = [&](double t) { return Eigen::Vector3d(p0 + Eigen::Vector3d(0.1 * std::sin(1.5 * t), 0.05 * std::sin(2.1 * t), 0.08 * (1 - std::cos(1.2 * t)))); };
            auto handq = [&](double t) { return Eigen::Quaterniond(Eigen::AngleAxisd(0.6 * std::sin(1.3 * t), Eigen::Vector3d(0.3, 0.5, 1).normalized())); };
            if (k % 14 == 0) { prev_held = held; prev_heldq = heldq; held = hand(t) + Eigen::Vector3d(n(rng), n(rng), n(rng)); heldq = rotationFromVector(Eigen::Vector3d(nr(rng), nr(rng), nr(rng))) * handq(t); t_s = t;
                if (estimate && k > 0) { est << (held - prev_held) / 0.014, rotationVector(heldq * prev_heldq.inverse()) / 0.014; } }
            const double age = std::min(t - t_s, 0.05);  // feedforward_timeout
            const Eigen::Vector3d tgt = held + est.head<3>() * age; const Eigen::Quaterniond tgtq = rotationFromVector(est.tail<3>() * age) * heldq;
            g.update(tgt, tgtq, est, 1, p, q, tw, acc);
            const V6 prev_acc = prev; track(s, tw, acc, prev, 1); prev = acc;
            if (t > 1.0) { sumj += (acc.head<3>() - prev_acc.head<3>()).squaredNorm() / 1e-6; err = std::max(err, (p - hand(t)).norm()); rerr = std::max(rerr, angle(handq(t), q)); cnt++; }
        }
        std::printf("72 Hz noisy: max error to hand %.2f mm / %.2f deg; lin a/j %.2f %.0f rot a/j %.2f %.0f, rms lin jerk %.0f\n", err * 1e3, rerr * 180 / M_PI, s.amax[0], s.jmax[0], s.amax[1], s.jmax[1], std::sqrt(sumj / cnt));
        if (estimate) check(err < 0.006 && rerr < 0.035, "tracks within 6 mm / 2 deg (twist from consecutive samples)");
        else check(err < 0.015 && rerr < 0.1, "tracks within 15 mm / 5.7 deg (held targets lag)");
        check(s.jmax[0] <= 200 * 1.01 && s.jmax[1] <= 400 * 1.01, "jerk bounded");
    }
    // 3) continuous rotation about z through +-pi (twist feed-forward 1 rad/s), lost packets (steps=3)
    {
        CartesianReferenceGenerator g;
        g.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), cfg());
        Eigen::Vector3d p; Eigen::Quaterniond q; V6 tw, acc;
        double worst = 0; int k = 0;
        for (; k < 10000; k += (k % 50 == 0) ? 3 : 1)
        {
            const double t = k * 1e-3;
            V6 ff = V6::Zero(); ff[5] = 1.0;
            const Eigen::Quaterniond qt(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitZ()));
            const uint64_t steps = (k % 50 == 0) ? 3 : 1;
            g.update(Eigen::Vector3d::Zero(), qt, ff, steps, p, q, tw, acc);
            if (t > 1.0) worst = std::max(worst, angle(qt, q));
        }
        std::printf("spin 10 rad: max lag %.2f deg, final wz %.3f rad/s\n", worst * 180 / M_PI, tw[5]);
        check(worst < 0.02, "follows through +-pi without flip (<1.1 deg)");
    }
    std::printf(fails ? "FAILED %d\n" : "ALL OK\n", fails);
    return fails;
}
