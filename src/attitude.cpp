#include "attitude.h"
#include <math.h>

namespace att {

Estimator est;

static constexpr float RAD2DEG = 57.2957795f;

static inline float wrap180(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

void Estimator::begin(float sampleRateHz) {
  fs_ = sampleRateHz > 1.0f ? sampleRateHz : 1600.0f;
  dt_ = 1.0f / fs_;
  roll_ = pitch_ = yaw_ = 0.0f;
  decim_ = 0;
  primed_ = false;
  rebuildLpf();
  startCalibration();
}

void Estimator::configure(const Config &cfg) {
  const float oldLpf = cfg_.gravLpfHz;
  cfg_ = cfg;
  cfg_.source = constrain((int)cfg_.source, 0, 2);
  (void)oldLpf;
  cfg_.gravLpfHz = constrain(cfg_.gravLpfHz, 0.2f, 40.0f);
  cfg_.kp        = constrain(cfg_.kp, 0.05f, 10.0f);
  cfg_.ki        = constrain(cfg_.ki, 0.0f, 1.0f);
  cfg_.tolG      = constrain(cfg_.tolG, 0.05f, 2.0f);
  rebuildLpf();
}

void Estimator::rebuildLpf() {
  // PT2: 12 dB/oitava. Em 3 Hz, uma vibracao de 100 Hz chega atenuada ~60 dB -
  // e o que faz a gravidade sobrar limpa no meio do chacoalho.
  const float k = bf::pt1FilterGain(cfg_.gravLpfHz * bf::CUTOFF_CORRECTION_PT2, dt_);
  for (int i = 0; i < 3; i++) gravLpf_[i].init(k);
  primed_ = false;
}

void Estimator::startCalibration() {
  calibrating_ = true;
  calCount_ = 0;
  calTarget_ = (uint32_t)(fs_ * 1.0f);   // ~1 s parado
  biasAcc_[0] = biasAcc_[1] = biasAcc_[2] = 0.0;
}

void Estimator::update(const float acc[3], const float gyr[3]) {
  haveGyro_ = (gyr != nullptr);

  accMag_ = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);

  // ---- 1) isola a gravidade no dominio da frequencia -----------------------
  float af[3];
  if (!primed_) {
    // parte do valor medido, senao a saida leva segundos para subir de zero
    for (int i = 0; i < 3; i++) { gravLpf_[i].state = acc[i]; gravLpf_[i].state1 = acc[i]; }
    primed_ = true;
  }
  for (int i = 0; i < 3; i++) af[i] = gravLpf_[i].apply(acc[i]);

  accMagF_ = sqrtf(af[0] * af[0] + af[1] * af[1] + af[2] * af[2]);
  if (accMagF_ > 1e-4f) {
    for (int i = 0; i < 3; i++) grav_[i] = af[i] / accMagF_;
  }
  // o que sobrou depois de tirar a gravidade e a vibracao propriamente dita
  const float vx = acc[0] - af[0], vy = acc[1] - af[1], vz = acc[2] - af[2];
  const float vmag = sqrtf(vx * vx + vy * vy + vz * vz);
  vib_ += 0.002f * (vmag - vib_);

  // ---- 2) confianca gradual, sem degraus ----------------------------------
  trust_ = 1.0f - fabsf(accMagF_ - 1.0f) / cfg_.tolG;
  trust_ = constrain(trust_, 0.0f, 1.0f);

  // atan2 e caro; a 1.6 kHz nao ha ganho nenhum em recalcular a cada amostra
  if ((decim_++ & 7) == 0) {
    rollAcc_  = atan2f(af[1], af[2]) * RAD2DEG;
    pitchAcc_ = atan2f(-af[0], sqrtf(af[1] * af[1] + af[2] * af[2])) * RAD2DEG;
  }

  if (!haveGyro_) {
    // Sem giroscopio sobra so o acelerometro filtrado. Ja vem suave do PT2,
    // entao pode ir direto; yaw fica indefinido.
    roll_ = rollAcc_;
    pitch_ = pitchAcc_;
    lastRate_[0] = lastRate_[1] = lastRate_[2] = 0.0f;
    return;
  }

  if (calibrating_) {
    for (int i = 0; i < 3; i++) biasAcc_[i] += gyr[i];
    if (++calCount_ >= calTarget_) {
      for (int i = 0; i < 3; i++) bias_[i] = (float)(biasAcc_[i] / calCount_);
      calibrating_ = false;
      yaw_ = 0.0f;
    }
    roll_ = rollAcc_;
    pitch_ = pitchAcc_;
    lastRate_[0] = lastRate_[1] = lastRate_[2] = 0.0f;
    return;
  }

  const float gx = gyr[0] - bias_[0];
  const float gy = gyr[1] - bias_[1];
  const float gz = gyr[2] - bias_[2];
  lastRate_[0] = gx; lastRate_[1] = gy; lastRate_[2] = gz;

  // ---- 3) complementar PI --------------------------------------------------
  // O termo P puxa o angulo para a referencia da gravidade; o termo I aprende
  // o offset do giroscopio. Quando a confianca cai, os dois recuam juntos e o
  // giroscopio - ja compensado - segura sozinho por muito mais tempo.
  const float errRoll  = wrap180(rollAcc_ - roll_);
  const float errPitch = wrap180(pitchAcc_ - pitch_);
  const float kp = cfg_.kp * trust_;
  const float ki = cfg_.ki * trust_;

  roll_  += (gx + kp * errRoll) * dt_;
  pitch_ += (gy + kp * errPitch) * dt_;
  yaw_   += gz * dt_;

  bias_[0] -= ki * errRoll * dt_;
  bias_[1] -= ki * errPitch * dt_;
  // limita o quanto o integrador pode andar, para um transitorio nao envenenar
  // a estimativa de offset
  bias_[0] = constrain(bias_[0], -20.0f, 20.0f);
  bias_[1] = constrain(bias_[1], -20.0f, 20.0f);

  roll_  = wrap180(roll_);
  pitch_ = constrain(pitch_, -90.0f, 90.0f);
  yaw_   = wrap180(yaw_);
}

State Estimator::state() const {
  State s;
  s.roll = roll_;
  s.pitch = pitch_;
  s.yaw = yaw_;
  s.gx = lastRate_[0];
  s.gy = lastRate_[1];
  s.gz = lastRate_[2];
  s.accMag = accMag_;
  s.accMagF = accMagF_;
  s.trust = trust_;
  s.vibG = vib_;
  for (int i = 0; i < 3; i++) { s.grav[i] = grav_[i]; s.bias[i] = bias_[i]; }
  s.calibrating = calibrating_;
  s.valid = haveGyro_;
  s.accFiltered = accFilt_;
  s.gyrFiltered = gyrFilt_;
  return s;
}

}  // namespace att
