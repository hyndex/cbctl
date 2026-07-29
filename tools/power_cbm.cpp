/* Power modules and the meter, via the SHIPPING drivers rather than a bench re-implementation.
 *
 * WHY THIS REPLACED HAND-ROLLED CODE
 *   The first version of this bench drove the ENR modules from a small ENR encoder written for
 *   the occasion. It looked right, it ACKed, it reported a perfect setpoint — and it never sent
 *   the output-enable, so the modules sat at 0 V holding a setpoint nobody had told them to
 *   apply. A charge that cannot happen, with no error anywhere.
 *
 *   cbmodules cannot make that mistake, because enable is not a separate message you can forget:
 *   it is a FIELD of GroupTarget, sits beside the voltage and current, and the manager owns the
 *   sequencing. That is the whole argument for using it here. cbmodules is also the code that
 *   actually runs on every charger in the fleet and has been audited against the ENR-S0
 *   datasheet; a bench that drives modules differently from production is testing the bench.
 *
 *   Same reasoning for cbmeter: METER_MEASURED is a provenance bit the EV-facing validator
 *   trusts, and it should be set because the shipping DCM3366 driver read the register, not
 *   because a bench Modbus helper agreed with itself.
 *
 * WHAT STAYED OURS
 *   The SocketCAN carriage below. cbmodules deliberately abstracts CAN behind CanTransport so
 *   it can run on an MCU with no sockets, so somebody has to provide the Linux end; it is
 *   forty lines and has no policy in it.
 *
 * SAFETY
 *   power_off() commands enable=false and is what every failure path calls. It is also safe to
 *   call before power_open() and twice in a row, because a cleanup path that can itself fail is
 *   not a cleanup path.
 */
#include <cbmodules/module_manager.hpp>
#include <cbmeter/drivers/dcm3366.hpp>
#include <cbmeter/transport.hpp>

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace {

uint32_t now_ms_mono()
{
    using namespace std::chrono;
    return static_cast<uint32_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

/* The Linux end of cbmodules' CanTransport. Non-blocking on purpose: poll_rx() asks for frames
 * until there are none, and a blocking read there would stall the charge loop that also has to
 * keep a 1500 ms feed alive on the PLC. */
class SocketCan final : public cbmodules::CanTransport {
public:
    ~SocketCan() override { if (fd_ >= 0) ::close(fd_); }

    bool open(const std::string &iface)
    {
        fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
        if (fd_ < 0) { return false; }
        struct ifreq ifr;
        std::memset(&ifr, 0, sizeof(ifr));
        std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
        if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) { return false; }
        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        return ::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) == 0;
    }

    bool send(const cbmodules::CanFrame &f) override
    {
        struct can_frame cf;
        std::memset(&cf, 0, sizeof(cf));
        cf.can_id = f.id | (f.extended ? CAN_EFF_FLAG : 0u);
        cf.can_dlc = static_cast<__u8>(f.dlc > 8 ? 8 : f.dlc);
        std::memcpy(cf.data, f.data, cf.can_dlc);
        return ::write(fd_, &cf, sizeof(cf)) == static_cast<ssize_t>(sizeof(cf));
    }

    bool receive(cbmodules::CanFrame &out) override
    {
        struct can_frame cf;
        const ssize_t n = ::read(fd_, &cf, sizeof(cf));
        if (n != static_cast<ssize_t>(sizeof(cf))) { return false; }
        out.extended = (cf.can_id & CAN_EFF_FLAG) != 0u;
        out.id = cf.can_id & (out.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
        out.dlc = cf.can_dlc;
        std::memcpy(out.data, cf.data, cf.can_dlc);
        return true;
    }

    uint32_t now_ms() const override { return now_ms_mono(); }

private:
    int fd_{-1};
};

/* RUNTIME TELEMETRY PROBING IS OPT-IN, AND THIS BENCH FORGOT TO OPT IN.
 *
 * cbmodules has a correct per-module round-robin READ — status / voltage / current /
 * available-current / temperature — but `runtime_probe_interval_ms` defaults to 0, which
 * DISABLES it. Default-constructing the manager therefore gives you a driver that COMMANDS
 * perfectly and never asks the modules anything after the startup scan.
 *
 * Counted on the wire over one 14 s run with the default config: 108 ALL_SET frames and 2 READ
 * frames, the two being startup_scan_validate. So `combined_current_a` never received a
 * measurement and read 0.0 A through an entire CurrentDemand phase while the DCM3366 downstream
 * measured 9.9 A and the vehicle's SoC climbed. That looked like a cbmodules telemetry bug and
 * was a missing line here.
 *
 * 500 ms matches cbcontroller/src/hardware_detect.cpp (kProbeIntervalMs); the shipping service
 * derives it from telemetry_period_ms in module_transport.cpp. Production enables it in both
 * paths — the fleet was never affected by this. */
cbmodules::ModuleManagerConfig bench_config()
{
    cbmodules::ModuleManagerConfig cfg;
    cfg.runtime_probe_interval_ms = 500u;
    return cfg;
}

struct Power {
    SocketCan can;
    cbmodules::ModuleManager mgr{bench_config()};
    std::shared_ptr<cbmeter::Transport> mtrans;
    std::unique_ptr<cbmeter::drivers::Dcm3366Driver> meter;
    bool open{false};
};

Power *g = nullptr;
const char *kGroup = "bench";

} /* namespace */

extern "C" {

/* addrs/naddr describe the modules physically on the bus. They are a parameter and not a
 * constant because reading them off the wire and hard-coding them is how a bench silently
 * drives the wrong module on the next charger. */
int power_open(const char *can_iface, const char *meter_dev, int meter_unit,
               const unsigned char *addrs, int naddr)
{
    if (g != nullptr) { return 0; }
    g = new Power();

    if (!g->can.open(can_iface ? can_iface : "can0")) { return -1; }
    g->mgr.set_transport(&g->can);

    std::vector<cbmodules::ModuleSpec> inv;
    for (int i = 0; i < naddr; ++i) {
        cbmodules::ModuleSpec s;
        s.id = "m" + std::to_string(addrs[i]);
        s.type = cbmodules::ModuleType::MaxwellEnr;
        s.address = addrs[i];
        s.group = 1;
        s.monitor_address = 1;
        s.rated_current_a = 60.0f;
        s.rated_power_kw = 30.0f;
        inv.push_back(s);
    }
    g->mgr.set_inventory(inv);

    /* A group must exist and own modules before a target means anything: set_group_target on an
     * empty group succeeds and delivers nothing, which is the same silent-nothing shape as the
     * missing enable. efficient_module_usage is OFF here — it allocates the minimum modules the
     * demand needs, which is right on a charger and wrong on a bench where "did every module I
     * expect actually come up" is the thing being tested. */
    cbmodules::GroupConfig gc;
    gc.id = kGroup;
    gc.min_modules = 1;
    gc.max_modules = static_cast<size_t>(naddr);
    gc.type_filter = cbmodules::ModuleType::MaxwellEnr;
    gc.efficient_module_usage = false;
    if (!g->mgr.upsert_group(gc)) { return -2; }

    /* Discovery is an EXPLICIT step, not something tick() eventually gets around to. ENR
     * modules do not broadcast when idle — they only answer a probe — so a manager that has
     * never scanned holds an inventory of modules it has never heard from, and every
     * allocation is refused with "undiscovered". Pumping the bus is not the same as asking. */
    const cbmodules::StartupScanReport scan = g->mgr.startup_scan_validate(5000u);
    std::fprintf(stderr, "cbmodules: scan expected=%zu discovered=%zu\n",
                 scan.expected_count, scan.modules.size());
    /* Print what the manager actually believes about each module. Guessing at the reject
     * string wastes a bench session; the raw status word says which bit did it. */
    for (const cbmodules::ModuleStateView &mv : g->mgr.module_states()) {
        std::fprintf(stderr,
                     "cbmodules: %s discovered=%d faulted=%d healthy=%d online=%d off=%d "
                     "status=0x%08X valid=%d faultmask=0x%08X V=%.1f\n",
                     mv.id.c_str(), (int)mv.discovered, (int)mv.telemetry.faulted,
                     (int)mv.telemetry.healthy, (int)mv.telemetry.online,
                     (int)mv.telemetry.module_off, mv.telemetry.raw_status_word,
                     (int)mv.telemetry.raw_status_word_valid, mv.telemetry.fault_class_mask,
                     (double)mv.telemetry.voltage_v);
    }

    int allocated = 0;
    for (const cbmodules::ModuleSpec &s2 : inv) {
        if (g->mgr.allocate_module(s2.id, kGroup)) {
            allocated++;
        } else {
            /* Name the gate rather than failing mutely — this is the counter that says
             * "the modules are on the bus but the manager will not drive them". */
            std::fprintf(stderr, "cbmodules: allocate %s -> %s REFUSED: %s\n",
                         s2.id.c_str(), kGroup, g->mgr.last_allocate_reject());
        }
    }
    if (allocated == 0) { return -3; }

    /* The meter is optional: a bench without one still charges, it just cannot claim
     * METER_MEASURED. Failing the whole run for it would be the wrong trade. */
    if (meter_dev != nullptr && meter_dev[0] != '\0') {
        g->mtrans = cbmeter::make_posix_transport(meter_dev);
        cbmeter::DriverConfig cfg;
        cfg.transport = g->mtrans;
        cfg.modbus_unit = meter_unit;
        /* SHORT ON PURPOSE — this read happens on the ONLY thread servicing either PLC.
         *
         * Dcm3366Driver::read() is one read_once plus, on a transient fault, a second: at the
         * production 1000 ms deadline a DEAD meter blocks ~2.1 s, which is 1.4x
         * CBPLC_LIMITS_STALE_MS and would drop BOTH guns' contactors at once. The driver's own
         * recovery_read() comment prices the same hazard ("would age the healthy gun's evidence
         * on the shared bus") and takes a sub-period deadline for it.
         *
         * The meter is a WITNESS here, not the control source — the modules answer the vehicle —
         * so a deadline short enough to be irrelevant to the loop costs only a skipped
         * corroboration. A healthy DCM3366 exchange at 9600 baud is ~30 ms; 150 ms is five times
         * that, and bounds the worst case to ~350 ms including the in-driver retry. */
        cfg.read_timeout = std::chrono::milliseconds(150);
        g->meter = std::make_unique<cbmeter::drivers::Dcm3366Driver>(cfg, cbmeter::drivers::Dcm3366Profile::Auto);
        (void)g->meter->probe();
    }
    g->open = true;
    return 0;
}

/* Enable travels WITH the setpoint. This is the bug-shaped hole the hand-rolled driver had. */
void power_set(double volts, double amps)
{
    if (g == nullptr) { return; }
    cbmodules::GroupTarget t;
    t.enable = true;
    t.voltage_v = static_cast<float>(volts);
    t.current_a = static_cast<float>(amps);
    (void)g->mgr.set_group_target(kGroup, t);
}

void power_off(void)
{
    if (g == nullptr) { return; }
    cbmodules::GroupTarget t; /* enable=false, 0 V, 0 A */
    (void)g->mgr.set_group_target(kGroup, t);
    g->mgr.tick(now_ms_mono());
}

void power_tick(void)
{
    if (g == nullptr) { return; }
    (void)g->mgr.poll_rx(0);
    g->mgr.tick(now_ms_mono());
}

/* 0 on a trustworthy reading, -1 otherwise. -1 is NOT zero volts: a stale poll and a real zero
 * are indistinguishable downstream and only one of them is safe to feed a charging session, so
 * the caller must withhold the sample rather than render this as 0.0. */
int power_read(double *v, double *a)
{
    if (g == nullptr) { return -1; }
    const cbmodules::GroupStatus st = g->mgr.group_status(kGroup);
    if (!st.valid || st.active_modules == 0 || st.telemetry_estimated) { return -1; }
    *v = st.combined_voltage_v;
    *a = st.combined_current_a;
    return 0;
}

/* Can this charger supply power AT ALL — a statement about the plant, not about how recently an
 * idle module answered a poll. Reported to the vehicle during ChargeParameterDiscovery, long
 * before any module has been asked to do anything. */
int power_available(void)
{
    if (g == nullptr) { return 0; }
    const cbmodules::GroupStatus st = g->mgr.group_status(kGroup);
    return (st.valid && st.assigned_modules > 0 && st.faulted_modules == 0) ? 1 : 0;
}

/* Why is the group not producing? assigned/active/confirmed_on/faulted separates "never
 * allocated" from "allocated but never enabled" from "enabled but not answering". */
void power_group_debug(int *assigned, int *active, int *confirmed_on, int *faulted,
                       double *req_v, double *req_a)
{
    if (g == nullptr) { return; }
    const cbmodules::GroupStatus st = g->mgr.group_status(kGroup);
    *assigned = (int)st.assigned_modules;
    *active = (int)st.active_modules;
    *confirmed_on = (int)st.confirmed_on_modules;
    *faulted = (int)st.faulted_modules;
    *req_v = st.requested_voltage_v;
    *req_a = st.requested_current_a;
}

int power_modules_on(void)
{
    if (g == nullptr) { return 0; }
    return static_cast<int>(g->mgr.group_status(kGroup).confirmed_on_modules);
}

int power_meter_read(int channel, double *v, double *a)
{
    if (g == nullptr || g->meter == nullptr) { return -1; }
    const auto r = g->meter->read(channel);
    if (!r) { return -1; }
    const cbmeter::MeterReading &m = *r;
    /* present AND valid. A register the meter never supplied and one that failed decoding are
     * different failures, and neither may be rendered as a measured zero. */
    if (!m.voltage_v.present || !m.voltage_v.valid ||
        !m.current_a.present || !m.current_a.valid) {
        return -1;
    }
    *v = m.voltage_v.value;
    *a = m.current_a.value;
    return 0;
}

void power_close(void)
{
    if (g == nullptr) { return; }
    power_off();
    delete g;
    g = nullptr;
}

} /* extern "C" */
