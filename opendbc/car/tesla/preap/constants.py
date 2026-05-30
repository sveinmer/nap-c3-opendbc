# Pre-AP pedal accel envelopes are defined in nap_conf.ACCEL_MAX_PROFILES
# (single source of truth). Personality → profile mapping lives in
# preap.interface.PERSONALITY_TO_PROFILE.
#
# Breakpoints are speed in m/s; values below are the follow-mode cap only.
ACCEL_PREAP_BP = [0.0, 1.3, 7.5, 15.0, 25.0, 40.0]  # m/s
#                  0    3    17    33    56    90  mph

# When following a lead car, cap positive accel to these values regardless
# of personality. Prevents overshoot → regen → overshoot oscillation.
# Open road uses the full personality profile; this only limits follow mode.
ACCEL_PREAP_FOLLOW = [0.3, 0.6, 0.9, 0.8, 0.7, 0.55]

# Outer LongControl tune. Fase B (2026-05-23) — port av Tinkla PCC PID-arkitektur
# etter rlog-bekreftet integral-windup-overshoot på drive 0000007f. Tidligere
# kp=0/kd=0 feedforward-dominant resulterte i 30-60s integral wash-out og
# +6 km/h lead-overshoot. Nå:
#   kp > 0: umiddelbar P-respons på error, demper integral-wandring.
#   kd > 0: demper d(error)/dt — særlig lead-akselerasjon-spike.
#   ki -20% mot V58-baseline for å redusere integral-windup gjennom ramp-up.
#   kf=1.0 beholdt: MPC-plan dominerer i steady-state.
# Konservativ Tier 1 — Tinkla bruker kp=0.75 ki=0.07 flat med kf=0.25; vår
# kf=1.0 gir mer baseline-gas, så ~27% av Tinkla's kp som startpunkt.
PEDAL_LONG_K_BP = [0.0, 3.0, 6.0, 35.0]
# Tier 2 (2026-05-24): gain-schedule kp + ki[3] boost etter drive 00000084
# viste ramp-tail-treghet ved set=81 km/h. Lav-fart 50-cruise allerede tight
# (Tier 1.5 p95_under=1.54 km/h ved set=50), så kp[0] urørt.
#
# Tier 2.1 (2026-05-24 sen ettermiddag): ki[3] 0.18 → 0.25 etter drive
# 00000088 (B6) viste persistent +0.5-1.8 km/h undershoot @ alle cruise-fart
# > 50 km/h. Drive 0000008c (B8) verifiserte fix: 100 km/h gap +1.08 → +0.24
# km/h (-78%), v_std 0.41 → 0.33.
#
# Tier 2.2 (2026-05-24 kveld): kp gain-schedule -20% + mild leak (0.998)
# re-introdusert etter Sveins B8-feedback: "rykking innen 1 km/h merkbar
# pga kraftig plant; aksepterer ±2-3 km/h gap for mindre rykking". Tier 2.1
# hadde knust gap men kp=0.35 ga skarp P-respons til MPC plan-jitter. kp_v
# redusert til [0.20, 0.22, 0.25, 0.28]. Mild leak (halveringstid ~3.5s)
# demper integral-cycling. Sammen forventes ~20-30% redusert jerk @ cruise
# med ca +0.5-1.0 km/h persistent gap som trade-off.
#
# Tier 2.3 (2026-05-27 sen kveld): ki[1] + ki[2] booster etter post-drive-
# analyse av 000000a0--9f852a8dd6 viste +3-6 km/h undershoot på ALLE
# setpoints (set=42 +4.82, set=55 +4.16, set=70 +3.95-5.67, set=84 +4.60).
# Tier 2.1 dekket kun ki[3] (>100 km/h-zone). Effektiv ki @ cruise-speeds
# var 0.13-0.16 (interp); Tier 2.1's 0.25-precedent closet gap. Tier 2.3
# bringer ki[1..2] proporsjonalt opp:
#   ki[1]: 0.06 → 0.10 (+67%)  — 10-20 km/h bånd
#   ki[2]: 0.08 → 0.14 (+75%)  — 20-50 km/h bånd
# Schedule [0.04, 0.10, 0.14, 0.25] er smooth-monotont. ki[0] urørt
# (0-3 m/s = stop/start, ikke testet 2026-05-27).
# Forventet effekt: +3-6 km/h gap → ±1 km/h @ alle cruise-speeds.
# Risiko: økt integral-windup ved aksellerasjon → mulig overshoot. Leak
# 0.998 + kp gain-schedule fra Tier 2.2 demper. Live-valider via tune-bridge
# FØR permanent deploy: `rig tune push --c3 ki=0.04,0.10,0.14,0.25`.
PEDAL_LONG_KP_V = [0.20, 0.22, 0.25, 0.28]
PEDAL_LONG_KI_V = [0.04, 0.10, 0.14, 0.25]
# Tier 1.5 (2026-05-24): kd deaktivert etter drive 00000081 viste cc.acc
# 2-5 Hz jitter ±0.30 m/s² ("av-på-av"). Rotårsak: error_rate * kd forsterker
# aEgo IMU-støy ~25× (DT_CTRL=0.01s → ±5 m/s²/s rate fra ±0.05 IMU-noise).
# Behold som tabell-skjelett for fremtidig re-enable hvis LP-filter på
# error_rate legges til. Verdiene som ble live-testet: [0.02, 0.04, 0.05, 0.05].
PEDAL_LONG_KD_V = [0.0, 0.0, 0.0, 0.0]

# Virtual DAS inner PID — sits inside carcontroller, corrects the
# feedforward output by tracking (a_limited - a_ego_future). Fase B
# (2026-05-23): aktivert kp+kd for å gi indre loopen umiddelbar respons,
# matchet outer-arkitekturen. Inner kp lavere enn outer fordi outer-output
# allerede gir P-respons på samme error-kjede.
VDAS_INNER_K_BP = [0.0, 5.0, 35.0]
VDAS_INNER_KP_V = [0.10, 0.10, 0.10]
# Halved from original [0.3, 0.2, 0.15] for milder PID-correction now that
# Tinkla S85 PEDAL_V LUT + drive-5c-data-driven FF-table are deployed.
# Less risk of integral windup amplifying FF-table residuals.
VDAS_INNER_KI_V = [0.15, 0.10, 0.05]
# Tier 1.5 (2026-05-24): inner kd deaktivert — samme grunn som outer
# (error_rate-noise-amplifisering på aEgo). Verdier live-testet: [0.02, 0.02, 0.02].
VDAS_INNER_KD_V = [0.0, 0.0, 0.0]

# Delay compensation: predict a_ego this far into the future using
# estimated jerk. Longer at highway speed where powertrain is slower.
VDAS_FUTURE_T_BP = [2.0, 5.0]
VDAS_FUTURE_T_V = [0.30, 0.55]

# a_ego low-pass filter time constant (seconds). Smooths IMU noise
# without adding too much phase lag. Matches Toyota's 0.25s RC.
VDAS_AEGO_FILTER_RC = 0.25

# Fase B (2026-05-23) — Tinkla PCC_module.py:334-335-paritet. Conditional
# hysteresis ±0.8 km/h rundt pedal_speed_kph-anchor: pedal-cmd fryser
# kun nær setpoint; utenfor band passerer MPC-respons fritt (lead-respons
# bevart). Match med compute_pedal_command default-band i
# opendbc/car/tesla/pedal/controller.py.
HYSTERESIS_BAND_KPH = 0.8

# Outer LongControl integral-leak per tick @100Hz. Tier 1.5 (2026-05-24):
# deaktivert (=1.0) etter drive 00000081 viste 5-14 km/h undershoot. Tier 2.2
# (2026-05-24 kveld): mild leak 0.998 re-introdusert etter drive 0000008c
# viste Tier 2.1 hadde knust persistent gap @ 100 km/h (+1.08 → +0.24 km/h)
# men subjektiv "rykking innen ±1 km/h" gjenstod på kraftig plant. ki[3]=0.25
# behold (Tier 2.1's seier); 0.998 leak demper integral-cycling-amplitude
# (halveringstid ~3.5s). Med ki[3]=0.25 og leak=0.998 settles steady-state
# integral ved I_ss = 1.25*err → persistent gap ca +0.5-1.0 km/h ved
# sustained cruise (innen Sveins ±2-3 km/h-smoothness-budsjett).
#
# Tier 2.4 (2026-05-28): leak 0.998 → 0.9995 etter Tier 2.3-live-test
# (drive 000000a6) viste at ki-boost alene ga kun -25% gap-reduksjon
# (forventet -40%), fordi leak begrenset integral-akkumulasjon. Tier 2.1
# uten leak (=1.0) klarte å lukke gap til +0.24 km/h; Tier 2.4 går halvveis
# tilbake mot Tier 2.1's leak-frihet:
#   leak 0.998 (halveringstid 3.5s) → 0.9995 (halveringstid ~14s)
#   integral-kapasitet ×4 → forventet gap ~+0.7-1 km/h (innen perfeksjons-mål ±1)
# Beholder Tier 2.3 ki-boost. Risk: økt v_std hvis jerk-cycling kommer
# tilbake (Tier 2.2's grunn til leak), men Tier 2.3-data viste v_std
# 0.1-1.5 km/h som har rom for litt mer.
#
# Tier 2.5 (2026-05-29 morgen, KUN via tune-bridge): leak 0.9995 → 0.9999
# live-validert i drive 000000a9. Gap @ 50 km/h 73s sustained: +1.42 km/h
# (-68% vs Tier 2.2). Gap @ 70 km/h 36s sustained: +1.50 km/h (-70% vs
# Tier 2.2). Begge v_std ~0.5 km/h. Tier 2.5 ble ALDRI permanent committet —
# kun live override, supersedert av Tier 2.6.
#
# Tier 2.6 (2026-05-29 formiddag): leak 0.9999 → 1.0 etter at vi oppdaget
# at NAPTune* keys ikke er i common/params_keys.h whitelist og blir slettet
# periodisk av openpilot's Params Manager. Tune-bridge-overrides er ikke
# robuste. Permanent SCP+reboot er eneste pålitelige path inntil
# params_keys.h utvides (separat sprint).
# leak=1.0 = Tier 2.1-precedence (lukket gap til +0.24 km/h @ 100 km/h).
# Med Tier 2.3 ki-boost over hele skjemaet forventes vi å treffe perfeksjons-
# mål (±1 km/h) over hele cruise-spekteret.
# Risk: jerk-cycling. Tier 2.5 live-data viste v_std 0.46-0.51 = god margin
# før vi rammer Tier 1's "av-på-av" (~v_std > 1.5 km/h).
INTEGRAL_LEAK = 1.0
