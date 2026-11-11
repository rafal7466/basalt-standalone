#include "bw/report.h"

#include <algorithm>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace bw {

std::string reportCss() {
  return
       ":root{--bg:#fbfaf7;--fg:#1d1d1b;--mut:#6b6a66;--card:#fff;--line:#e4e1da;--gt:#1d1d1b;"
       "--gt2:#7a7a7a;--gt3:#b0aca3;--est:#c2410c;--est2:#0f766e;--est3:#4338ca;--err:#c2410c;"
       "--acc:#c2410c}"
       "@media (prefers-color-scheme:dark){:root{--bg:#141412;--fg:#ecebe6;--mut:#9c9a93;--card:#1d1d1a;"
       "--line:#33322e;--gt:#ecebe6;--gt2:#a3a29c;--gt3:#6e6d68;--est:#fb923c;--est2:#2dd4bf;--est3:#a5b4fc;"
       "--err:#fb923c;--acc:#fb923c}}"
       "body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.55 'Segoe UI',system-ui,sans-serif}"
       "main{max-width:1060px;margin:0 auto;padding:28px 16px 60px}"
       "h1{font-size:26px;margin:0 0 4px}h2{font-size:18px;margin:34px 0 10px;border-bottom:1px solid var(--line);padding-bottom:6px}"
       ".sub{color:var(--mut);margin:0 0 22px}"
       ".kpis{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:12px}"
       ".kpi{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px}"
       ".kpi b{display:block;font-size:24px;font-variant-numeric:tabular-nums}.kpi span{color:var(--mut);font-size:13px}"
       ".kpi.main b{color:var(--acc)}"
       "table{border-collapse:collapse;width:100%;background:var(--card);border:1px solid var(--line);border-radius:8px;overflow:hidden;font-variant-numeric:tabular-nums}"
       "th,td{padding:7px 10px;border-bottom:1px solid var(--line);text-align:right}th:first-child,td:first-child{text-align:left}"
       "th{font-weight:600;color:var(--mut);font-size:13px}"
       ".grid2{display:grid;grid-template-columns:repeat(auto-fill,minmax(420px,1fr));gap:14px}"
       "@media (max-width:520px){.grid2{grid-template-columns:1fr}}"
       ".card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:8px}"
       "svg.plot{width:100%;height:auto;display:block}"
       "svg .grid{stroke:var(--line);stroke-width:1}svg .frame{fill:none;stroke:var(--mut);stroke-width:1}"
       "svg text{fill:var(--fg);font-family:inherit}svg .tk{font-size:11px;fill:var(--mut)}svg .al{font-size:12px}"
       "svg .pt{font-size:14px;font-weight:600}svg .lg{font-size:12px}"
       "code{background:var(--line);padding:1px 5px;border-radius:4px;font-size:13px}"
       ".note{color:var(--mut);font-size:13px}";
}

std::string fmt(double v, int prec) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(prec) << v;
  return ss.str();
}

// Simple SVG line plot. equal_aspect keeps metric proportions (trajectories).
std::string svgPlot(const std::vector<Series>& series, const std::string& title,
                    const std::string& xlabel, const std::string& ylabel, bool equal_aspect,
                    int W, int H) {
  double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300;
  for (const auto& s : series)
    for (size_t i = 0; i < s.x.size(); i++) {
      xmin = std::min(xmin, s.x[i]);
      xmax = std::max(xmax, s.x[i]);
      ymin = std::min(ymin, s.y[i]);
      ymax = std::max(ymax, s.y[i]);
    }
  if (xmax <= xmin) xmax = xmin + 1;
  if (ymax <= ymin) ymax = ymin + 1;
  const int ml = 70, mr = 16, mt = 34, mb = 48;
  const double pw = W - ml - mr, ph = H - mt - mb;
  double px = (xmax - xmin) * 0.04, py = (ymax - ymin) * 0.06;
  xmin -= px;
  xmax += px;
  ymin -= py;
  ymax += py;
  if (equal_aspect) {
    const double sx = (xmax - xmin) / pw, sy = (ymax - ymin) / ph;
    if (sx > sy) {
      const double c = 0.5 * (ymin + ymax), h = sx * ph;
      ymin = c - h / 2;
      ymax = c + h / 2;
    } else {
      const double c = 0.5 * (xmin + xmax), w = sy * pw;
      xmin = c - w / 2;
      xmax = c + w / 2;
    }
  }
  auto X = [&](double x) { return ml + (x - xmin) / (xmax - xmin) * pw; };
  auto Y = [&](double y) { return mt + ph - (y - ymin) / (ymax - ymin) * ph; };

  std::ostringstream o;
  o << "<svg viewBox='0 0 " << W << " " << H << "' xmlns='http://www.w3.org/2000/svg' "
    << "class='plot' role='img' aria-label='" << title << "'>";
  o << "<text x='" << W / 2 << "' y='20' text-anchor='middle' class='pt'>" << title << "</text>";
  // grid + ticks
  auto nice = [](double range) {
    const double raw = range / 6;
    const double mag = std::pow(10, std::floor(std::log10(raw)));
    const double r = raw / mag;
    return (r < 1.5 ? 1 : r < 3 ? 2 : r < 7 ? 5 : 10) * mag;
  };
  const double dx = nice(xmax - xmin), dy = nice(ymax - ymin);
  auto decimals = [](double d) { return std::max(0, (int)std::ceil(-std::log10(d) - 1e-9)); };
  auto clean = [](double v, double d) { return std::abs(v) < 1e-9 * d ? 0.0 : v; };
  for (double x = std::ceil(xmin / dx) * dx; x <= xmax; x += dx) {
    o << "<line x1='" << X(x) << "' y1='" << mt << "' x2='" << X(x) << "' y2='" << mt + ph
      << "' class='grid'/>";
    o << "<text x='" << X(x) << "' y='" << mt + ph + 16 << "' text-anchor='middle' class='tk'>"
      << fmt(clean(x, dx), decimals(dx)) << "</text>";
  }
  for (double y = std::ceil(ymin / dy) * dy; y <= ymax; y += dy) {
    o << "<line x1='" << ml << "' y1='" << Y(y) << "' x2='" << ml + pw << "' y2='" << Y(y)
      << "' class='grid'/>";
    o << "<text x='" << ml - 6 << "' y='" << Y(y) + 4 << "' text-anchor='end' class='tk'>"
      << fmt(clean(y, dy), decimals(dy)) << "</text>";
  }
  o << "<rect x='" << ml << "' y='" << mt << "' width='" << pw << "' height='" << ph
    << "' class='frame'/>";
  o << "<text x='" << ml + pw / 2 << "' y='" << H - 10 << "' text-anchor='middle' class='al'>"
    << xlabel << "</text>";
  o << "<text transform='translate(16," << mt + ph / 2 << ") rotate(-90)' text-anchor='middle' "
    << "class='al'>" << ylabel << "</text>";
  for (const auto& s : series) {
    o << "<polyline fill='none' style='stroke:" << s.color << "' stroke-width='" << s.width << "'"
      << (s.dashed ? " stroke-dasharray='5 4'" : "") << " points='";
    const size_t step = std::max<size_t>(1, s.x.size() / 2500);
    for (size_t i = 0; i < s.x.size(); i += step) o << fmt(X(s.x[i]), 1) << "," << fmt(Y(s.y[i]), 1) << " ";
    o << "'/>";
  }
  // legend
  int ly = mt + 14;
  for (const auto& s : series) {
    if (s.label.empty()) continue;
    o << "<line x1='" << ml + 12 << "' y1='" << ly - 4 << "' x2='" << ml + 34 << "' y2='" << ly - 4
      << "' style='stroke:" << s.color << "' stroke-width='3'" << (s.dashed ? " stroke-dasharray='5 4'" : "")
      << "/>";
    o << "<text x='" << ml + 40 << "' y='" << ly << "' class='lg'>" << s.label << "</text>";
    ly += 18;
  }
  o << "</svg>";
  return o.str();
}

namespace {

std::string statsRow(const std::string& name, const ErrorStats& s, int prec = 4) {
  return "<tr><td>" + name + "</td><td>" + fmt(s.rmse, prec) + "</td><td>" + fmt(s.mean, prec) +
         "</td><td>" + fmt(s.median, prec) + "</td><td>" + fmt(s.std, prec) + "</td><td>" +
         fmt(s.min, prec) + "</td><td>" + fmt(s.max, prec) + "</td><td>" +
         std::to_string(s.count) + "</td></tr>";
}

std::string mdRow(const std::string& name, const ErrorStats& s, int prec = 4) {
  return "| " + name + " | " + fmt(s.rmse, prec) + " | " + fmt(s.mean, prec) + " | " +
         fmt(s.median, prec) + " | " + fmt(s.std, prec) + " | " + fmt(s.min, prec) + " | " +
         fmt(s.max, prec) + " | " + std::to_string(s.count) + " |\n";
}

}  // namespace

void writeReport(const std::string& out_dir, const RunInfo& info, const EvalResult& ev,
                 const std::vector<GtPose>& est_traj) {
  namespace fs = std::filesystem;
  fs::create_directories(out_dir);

  const ErrorStats of = computeStats(info.of_ms);
  const ErrorStats vio = computeStats(info.vio_ms);
  std::vector<double> tot(info.of_ms.size());
  for (size_t i = 0; i < tot.size(); i++) tot[i] = info.of_ms[i] + info.vio_ms[i];
  const ErrorStats total = computeStats(tot);
  std::vector<double> lms(info.num_landmarks.begin(), info.num_landmarks.end());
  const ErrorStats lm = computeStats(lms);
  std::vector<double> trk(info.num_tracked.begin(), info.num_tracked.end());
  const ErrorStats tr = computeStats(trk);

  std::time_t now = std::time(nullptr);
  char date[64];
  std::strftime(date, sizeof(date), "%Y-%m-%d %H:%M", std::localtime(&now));

  // ---------- plots
  Series gx{{}, {}, "var(--gt)", "Ground truth (Leica/Vicon)", 2.2, true};
  Series ex{{}, {}, "var(--est)", "Basalt-Win VIO (SE3 aligned)", 1.6};
  Series gz = gx, ez = ex;
  for (size_t i = 0; i < ev.p_gt.size(); i++) {
    gx.x.push_back(ev.p_gt[i].x());
    gx.y.push_back(ev.p_gt[i].y());
    ex.x.push_back(ev.p_est_aligned[i].x());
    ex.y.push_back(ev.p_est_aligned[i].y());
    gz.x.push_back(ev.p_gt[i].x());
    gz.y.push_back(ev.p_gt[i].z());
    ez.x.push_back(ev.p_est_aligned[i].x());
    ez.y.push_back(ev.p_est_aligned[i].z());
  }
  const std::string plot_xy = svgPlot({gx, ex}, "Trajectory, top view (x-y)", "x [m]", "y [m]", true);
  const std::string plot_xz = svgPlot({gz, ez}, "Trajectory, side view (x-z)", "x [m]", "z [m]", true);

  Series ate{{}, {}, "var(--err)", "", 1.4};
  const double t0 = ev.t_ns.empty() ? 0 : ev.t_ns.front() * 1e-9;
  for (size_t i = 0; i < ev.ate_err.size(); i++) {
    ate.x.push_back(ev.t_ns[i] * 1e-9 - t0);
    ate.y.push_back(ev.ate_err[i]);
  }
  const std::string plot_ate = svgPlot({ate}, "Absolute trajectory error over time", "time [s]",
                                       "ATE [m]", false);

  std::vector<Series> axes;
  const char* names[3] = {"x", "y", "z"};
  const char* gcol[3] = {"var(--gt)", "var(--gt2)", "var(--gt3)"};
  const char* ecol[3] = {"var(--est)", "var(--est2)", "var(--est3)"};
  for (int k = 0; k < 3; k++) {
    Series g{{}, {}, gcol[k], std::string("GT ") + names[k], 2.0, true};
    Series e{{}, {}, ecol[k], std::string("VIO ") + names[k], 1.4};
    for (size_t i = 0; i < ev.p_gt.size(); i++) {
      g.x.push_back(ev.t_ns[i] * 1e-9 - t0);
      g.y.push_back(ev.p_gt[i][k]);
      e.x.push_back(ev.t_ns[i] * 1e-9 - t0);
      e.y.push_back(ev.p_est_aligned[i][k]);
    }
    axes.push_back(g);
    axes.push_back(e);
  }
  const std::string plot_axes = svgPlot(axes, "Position per axis", "time [s]", "position [m]", false, 640, 420);

  Series tof{info.frame_t_s, info.of_ms, "var(--gt2)", "optical flow", 1.0};
  Series tvio{info.frame_t_s, info.vio_ms, "var(--est)", "VIO optimization + marginalization", 1.0};
  const std::string plot_time = svgPlot({tof, tvio}, "Processing time per frame", "time [s]", "ms", false);

  Series slm{info.frame_t_s, {}, "var(--est)", "landmarks in window", 1.2};
  Series str{info.frame_t_s, {}, "var(--gt2)", "tracked points cam0", 1.2};
  for (size_t i = 0; i < info.num_landmarks.size(); i++) {
    slm.y.push_back(info.num_landmarks[i]);
    str.y.push_back(info.num_tracked[i]);
  }
  const std::string plot_lm = svgPlot({str, slm}, "Tracked points and landmarks", "time [s]", "count", false);

  Series bgx, bgy, bgz, bax, bay, baz;
  bgx = {{}, {}, "var(--est)", "bg x", 1.3};
  bgy = {{}, {}, "var(--est2)", "bg y", 1.3};
  bgz = {{}, {}, "var(--est3)", "bg z", 1.3};
  bax = {{}, {}, "var(--est)", "ba x", 1.3};
  bay = {{}, {}, "var(--est2)", "ba y", 1.3};
  baz = {{}, {}, "var(--est3)", "ba z", 1.3};
  for (size_t i = 0; i < info.bias_gyro.size(); i++) {
    const double t = info.frame_t_s[i];
    bgx.x.push_back(t); bgx.y.push_back(info.bias_gyro[i].x());
    bgy.x.push_back(t); bgy.y.push_back(info.bias_gyro[i].y());
    bgz.x.push_back(t); bgz.y.push_back(info.bias_gyro[i].z());
    bax.x.push_back(t); bax.y.push_back(info.bias_accel[i].x());
    bay.x.push_back(t); bay.y.push_back(info.bias_accel[i].y());
    baz.x.push_back(t); baz.y.push_back(info.bias_accel[i].z());
  }
  const std::string plot_bg = svgPlot({bgx, bgy, bgz}, "Estimated gyro bias (on top of calibration)", "time [s]", "rad/s", false);
  const std::string plot_ba = svgPlot({bax, bay, baz}, "Estimated accel bias (on top of calibration)", "time [s]", "m/s^2", false);

  // ---------- HTML
  std::ofstream h((fs::path(out_dir) / "report.html").string());
  h << "<!doctype html><html lang='en'><head><meta charset='utf-8'>"
       "<meta name='viewport' content='width=device-width,initial-scale=1'>"
       "<title>Basalt-Win EuRoC " << info.sequence_name << "</title><style>"
       << reportCss() << "</style></head><body><main>";
  h << "<h1>Basalt-Win VIO &middot; EuRoC " << info.sequence_name << "</h1>";
  h << "<p class='sub'>Stereo-inertial odometry, native Windows C++ port of Basalt (no ROS, no Linux). "
    << "Generated " << date << ".</p>";
  h << "<div class='kpis'>"
    << "<div class='kpi main'><b>" << fmt(ev.ate_se3.rmse, 3) << " m</b><span>ATE RMSE (SE3 aligned)</span></div>"
    << "<div class='kpi'><b>" << fmt(ev.ate_sim3.rmse, 3) << " m</b><span>ATE RMSE (Sim3, scale "
    << fmt(ev.sim3_scale, 3) << ")</span></div>"
    << "<div class='kpi'><b>" << fmt(ev.rpe_trans_1s.rmse, 3) << " m</b><span>RPE trans RMSE / 1 s</span></div>"
    << "<div class='kpi'><b>" << fmt(ev.rpe_rot_1s_deg.rmse, 3) << "&deg;</b><span>RPE rot RMSE / 1 s</span></div>"
    << "<div class='kpi'><b>" << fmt(total.mean, 1) << " ms</b><span>mean time per stereo frame</span></div>"
    << "<div class='kpi'><b>" << fmt(info.num_frames / std::max(1e-9, info.wall_time_s), 0)
    << " Hz</b><span>throughput (camera is 20 Hz)</span></div>"
    << "</div>";

  h << "<h2>Trajectory</h2><div class='grid2'><div class='card'>" << plot_xy << "</div><div class='card'>"
    << plot_xz << "</div></div>";
  h << "<h2>Accuracy</h2><table><tr><th>metric</th><th>RMSE</th><th>mean</th><th>median</th><th>std</th>"
       "<th>min</th><th>max</th><th>N</th></tr>"
    << statsRow("ATE, SE3 alignment [m]", ev.ate_se3) << statsRow("ATE, Sim3 alignment [m]", ev.ate_sim3)
    << statsRow("RPE translation, &Delta;=1 s [m]", ev.rpe_trans_1s)
    << statsRow("RPE rotation, &Delta;=1 s [deg]", ev.rpe_rot_1s_deg) << "</table>";
  h << "<p class='note'>Estimated poses are the IMU-frame state of the newest frame after each optimization "
       "(as Basalt's <code>vio</code> tool saves them), associated to ground truth "
       "(<code>state_groundtruth_estimate0</code>) by timestamp with linear interpolation. "
       "Ground-truth path length: " << fmt(ev.gt_length_m, 1) << " m over " << fmt(ev.duration_s, 1)
    << " s. ATE as a share of path length: " << fmt(100.0 * ev.ate_se3.rmse / std::max(1e-9, ev.gt_length_m), 2)
    << "%.</p>";
  h << "<p class='note'>Reference: the original Basalt VIO reports 0.07 m RMS ATE on MH_01 "
       "(Usenko et al., <i>Visual-Inertial Mapping with Non-Linear Factor Recovery</i>, RA-L 2020).</p>";
  h << "<div class='grid2'><div class='card'>" << plot_ate << "</div><div class='card'>" << plot_axes
    << "</div></div>";

  h << "<h2>Runtime</h2><table><tr><th>stage (per stereo frame)</th><th>mean ms</th><th>median ms</th>"
       "<th>max ms</th></tr>"
    << "<tr><td>Optical flow (pyramids, tracking, detection, stereo)</td><td>" << fmt(of.mean, 2)
    << "</td><td>" << fmt(of.median, 2) << "</td><td>" << fmt(of.max, 2) << "</td></tr>"
    << "<tr><td>VIO (LM optimization + marginalization)</td><td>" << fmt(vio.mean, 2) << "</td><td>"
    << fmt(vio.median, 2) << "</td><td>" << fmt(vio.max, 2) << "</td></tr>"
    << "<tr><td>Total</td><td>" << fmt(total.mean, 2) << "</td><td>" << fmt(total.median, 2)
    << "</td><td>" << fmt(total.max, 2) << "</td></tr></table>";
  h << "<p class='note'>Image loading from disk is excluded. Wall time for the whole sequence: "
    << fmt(info.wall_time_s, 1) << " s for " << info.num_frames << " stereo frames. "
    << "Landmarks in window: mean " << fmt(lm.mean, 0) << ", max " << fmt(lm.max, 0)
    << ". Tracked points (cam0): mean " << fmt(tr.mean, 0) << ". Keyframes created: "
    << info.num_keyframes << ".</p>";
  h << "<div class='grid2'><div class='card'>" << plot_time << "</div><div class='card'>" << plot_lm
    << "</div></div><h2>IMU biases</h2><div class='grid2'><div class='card'>" << plot_bg
    << "</div><div class='card'>" << plot_ba << "</div></div>";

  h << "<h2>Setup</h2><table>"
    << "<tr><td>Dataset</td><td>" << info.dataset_path << "</td></tr>"
    << "<tr><td>Frames processed</td><td>" << info.num_frames << "</td></tr>"
    << "<tr><td>Camera model</td><td>Double Sphere (Basalt EuRoC calibration)</td></tr>"
    << "<tr><td>Front-end</td><td>Basalt frame-to-frame patch flow, 3 pyramid levels, 50 px grid, "
       "SE2 patch alignment, fwd-bwd check, epipolar check</td></tr>"
    << "<tr><td>Back-end</td><td>3 states + 7 keyframes, LM (&le;7 it), Schur complement, "
       "marginalization with lost-landmark marginalization</td></tr>"
    << "<tr><td>Build</td><td>" << info.build_info << "</td></tr>"
    << "<tr><td>Estimated trajectory</td><td>" << info.trajectory_file << "</td></tr>"
    << "</table></main></body></html>";

  // ---------- Markdown
  std::ofstream m((fs::path(out_dir) / "report.md").string());
  m << "# Basalt-Win VIO - EuRoC " << info.sequence_name << "\n\nGenerated " << date << "\n\n";
  m << "Dataset: `" << info.dataset_path << "`  \nFrames: " << info.num_frames
    << "  \nKeyframes created: " << info.num_keyframes << "  \nBuild: " << info.build_info << "\n\n";
  m << "## Accuracy\n\n| metric | RMSE | mean | median | std | min | max | N |\n|---|---|---|---|---|---|---|---|\n"
    << mdRow("ATE SE3 [m]", ev.ate_se3) << mdRow("ATE Sim3 [m]", ev.ate_sim3)
    << mdRow("RPE trans 1 s [m]", ev.rpe_trans_1s) << mdRow("RPE rot 1 s [deg]", ev.rpe_rot_1s_deg);
  m << "\nSim3 scale: " << fmt(ev.sim3_scale, 4) << "  \nGT path length: " << fmt(ev.gt_length_m, 2)
    << " m, duration " << fmt(ev.duration_s, 1) << " s  \nReference: original Basalt VIO reports 0.07 m RMS ATE on MH_01 (Usenko et al., RA-L 2020)\n\n";
  m << "## Runtime (ms per stereo frame)\n\n| stage | mean | median | max |\n|---|---|---|---|\n"
    << "| optical flow | " << fmt(of.mean, 2) << " | " << fmt(of.median, 2) << " | " << fmt(of.max, 2) << " |\n"
    << "| VIO | " << fmt(vio.mean, 2) << " | " << fmt(vio.median, 2) << " | " << fmt(vio.max, 2) << " |\n"
    << "| total | " << fmt(total.mean, 2) << " | " << fmt(total.median, 2) << " | " << fmt(total.max, 2) << " |\n\n"
    << "Wall time: " << fmt(info.wall_time_s, 1) << " s\n";

  // ---------- metrics as CSV (machine readable)
  std::ofstream c((fs::path(out_dir) / "metrics.csv").string());
  c << "metric,rmse,mean,median,std,min,max,count\n";
  auto crow = [&](const std::string& n, const ErrorStats& s) {
    c << n << "," << s.rmse << "," << s.mean << "," << s.median << "," << s.std << "," << s.min << ","
      << s.max << "," << s.count << "\n";
  };
  crow("ate_se3_m", ev.ate_se3);
  crow("ate_sim3_m", ev.ate_sim3);
  crow("rpe_trans_1s_m", ev.rpe_trans_1s);
  crow("rpe_rot_1s_deg", ev.rpe_rot_1s_deg);
  crow("time_optical_flow_ms", of);
  crow("time_vio_ms", vio);
  crow("time_total_ms", total);
  (void)est_traj;
}

}  // namespace bw
