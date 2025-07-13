// === Vector3 class ===
class Vector3 {
  constructor(x=0,y=0,z=0){ this.x=x; this.y=y; this.z=z; }
  add(v){ return new Vector3(this.x+v.x, this.y+v.y, this.z+v.z); }
  subtract(v){ return new Vector3(this.x-v.x, this.y-v.y, this.z-v.z); }
  multiplyScalar(s){ return new Vector3(this.x*s, this.y*s, this.z*s); }
  length(){ return Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z); }
  normalize(){ 
    const len = this.length(); 
    return len>0 ? this.multiplyScalar(1/len) : new Vector3(0,0,0); 
  }
  clone(){ return new Vector3(this.x,this.y,this.z); }
  static zero(){ return new Vector3(0,0,0); }
}

// === Kalman filter cho 1 trục ===
class KalmanFilter {
  constructor(Q=0.005, R=0.1){
    this.Q = Q; // Process noise covariance
    this.R = R; // Measurement noise covariance
    this.P = 1; // Estimation error covariance
    this.K = 0; // Kalman gain
    this.x = 0; // Estimated value
  }
  update(measurement){
    this.P = this.P + this.Q;
    this.K = this.P / (this.P + this.R);
    this.x = this.x + this.K*(measurement - this.x);
    this.P = (1 - this.K)*this.P;
    return this.x;
  }
}

// === Kalman filter cho 3D vector ===
class KalmanFilter3D {
  constructor(){
    this.xFilter = new KalmanFilter();
    this.yFilter = new KalmanFilter();
    this.zFilter = new KalmanFilter();
  }
  update(vec3){
    return new Vector3(
      this.xFilter.update(vec3.x),
      this.yFilter.update(vec3.y),
      this.zFilter.update(vec3.z)
    );
  }
}

// === AimLock Engine ===
class AimLockHeadLock {
  constructor(config = {}){
    this.config = Object.assign({
      leadTime: 0.0001, // Thời gian dự đoán trước (giây)
      smoothFactor: 0.85, // Hệ số mượt
      maxLeadDistance: 1000.0, // Giới hạn khoảng cách dự đoán
      recoilSmoothFactor: 1.0 // Hệ số mượt cho recoil compensation
    }, config);

    this.prevEnemyPos = null;
    this.enemyVelocity = Vector3.zero();
    this.smoothAimPos = Vector3.zero();
    this.smoothedRecoilOffset = Vector3.zero();
    this.lastUpdateTime = null;

    this.kalmanFilter = new KalmanFilter3D();
  }

  // Cập nhật velocity của mục tiêu
  updateEnemyVelocity(currentPos, currentTime){
    if(!this.prevEnemyPos){
      this.prevEnemyPos = currentPos.clone();
      this.lastUpdateTime = currentTime;
      return;
    }
    const dt = (currentTime - this.lastUpdateTime) / 1000;
    if(dt <= 0) return;
    const newVel = currentPos.subtract(this.prevEnemyPos).multiplyScalar(1/dt);

    // Smooth velocity
    this.enemyVelocity = new Vector3(
      this.enemyVelocity.x * this.config.smoothFactor + newVel.x * (1 - this.config.smoothFactor),
      this.enemyVelocity.y * this.config.smoothFactor + newVel.y * (1 - this.config.smoothFactor),
      this.enemyVelocity.z * this.config.smoothFactor + newVel.z * (1 - this.config.smoothFactor)
    );

    this.prevEnemyPos = currentPos.clone();
    this.lastUpdateTime = currentTime;
  }

  // Cập nhật recoil offset với smoothing
  updateRecoilOffset(newRecoilOffset){
    this.smoothedRecoilOffset = new Vector3(
      this.smoothedRecoilOffset.x * this.config.recoilSmoothFactor + newRecoilOffset.x * (1 - this.config.recoilSmoothFactor),
      this.smoothedRecoilOffset.y * this.config.recoilSmoothFactor + newRecoilOffset.y * (1 - this.config.recoilSmoothFactor),
      this.smoothedRecoilOffset.z * this.config.recoilSmoothFactor + newRecoilOffset.z * (1 - this.config.recoilSmoothFactor)
    );
  }

  // Dự đoán vị trí đầu mục tiêu dựa trên velocity và leadTime
  predictHeadPosition(headPos, leadTime){
    let predicted = headPos.add(this.enemyVelocity.multiplyScalar(leadTime));
    const offsetVec = predicted.subtract(headPos);
    if(offsetVec.length() > this.config.maxLeadDistance){
      predicted = headPos.add(offsetVec.normalize().multiplyScalar(this.config.maxLeadDistance));
    }
    return predicted;
  }

  // Smoothing aim để không giật
  smoothAim(targetPos){
    this.smoothAimPos = new Vector3(
      this.smoothAimPos.x * this.config.smoothFactor + targetPos.x * (1 - this.config.smoothFactor),
      this.smoothAimPos.y * this.config.smoothFactor + targetPos.y * (1 - this.config.smoothFactor),
      this.smoothAimPos.z * this.config.smoothFactor + targetPos.z * (1 - this.config.smoothFactor)
    );
    return this.smoothAimPos.clone();
  }

  // Hàm lấy tọa độ aim cuối cùng (áp dụng recoil, smoothing, Kalman filter)
  getAimPosition(playerPos, boneHeadPos, recoilOffset, currentTime){
    this.updateEnemyVelocity(boneHeadPos, currentTime);
    this.updateRecoilOffset(recoilOffset);
    let predictedHead = this.predictHeadPosition(boneHeadPos, this.config.leadTime);

    // Ứng dụng Kalman Filter để ổn định vị trí predicted
    predictedHead = this.kalmanFilter.update(predictedHead);

    // Trừ recoil offset
    predictedHead = predictedHead.subtract(this.smoothedRecoilOffset);

    // Lerp mượt theo smoothAimPos
    return this.smoothAim(predictedHead);
  }
}

// === HitDetectCollider: hình cầu vùng bone head ===
function hitDetectColliderBoneHead(rayOrigin, rayDirection, options = {}) {
  const bindpose = options.bindpose || {
    e00: -1.34559613e-13, e01: 8.881784e-14, e02: -1.0, e03: 0.487912,
    e10: -2.84512817e-6,  e11: -1.0,        e12: 8.881784e-14, e13: -2.842171e-14,
    e20: -1.0,            e21: 2.84512817e-6, e22: -1.72951931e-13, e23: 0.0,
  };

  // Vị trí bone head hiện tại (world space)
  const headPos = options.position || new Vector3(-0.0456970781, -0.004478302, -0.0200432576);
  const radius = options.radius || 1.0; // Bán kính collider đầu (có thể điều chỉnh)

  // Ma trận 3x4 từ bindpose (để chuyển tọa độ vị trí bindpose sang thế giới)
  const mat = [
    bindpose.e00, bindpose.e01, bindpose.e02, bindpose.e03,
    bindpose.e10, bindpose.e11, bindpose.e12, bindpose.e13,
    bindpose.e20, bindpose.e21, bindpose.e22, bindpose.e23
  ];

  // Hàm nhân ma trận 3x4 với vector (chuyển position bindpose thành vị trí thế giới)
  function multiplyMatrixVec(m, v) {
    return new Vector3(
      m[0]*v.x + m[1]*v.y + m[2]*v.z + m[3],
      m[4]*v.x + m[5]*v.y + m[6]*v.z + m[7],
      m[8]*v.x + m[9]*v.y + m[10]*v.z + m[11]
    );
  }

  // Tính vị trí thực của bone head trên thế giới
  const worldHeadPos = multiplyMatrixVec(mat, headPos);

  // Vector từ ray origin đến center collider
  const oc = rayOrigin.subtract(worldHeadPos);
  const dir = rayDirection.normalize();

  // Giải phương trình đường thẳng - hình cầu
  const a = dir.x*dir.x + dir.y*dir.y + dir.z*dir.z;
  const b = 2 * (oc.x*dir.x + oc.y*dir.y + oc.z*dir.z);
  const c = oc.x*oc.x + oc.y*oc.y + oc.z*oc.z - radius*radius;

  const discriminant = b*b - 4*a*c;
  return discriminant >= 0;
}

// === API giả lập setAim(x,y,z) ===
// Bạn thay thế hàm này bằng API thực tế của bạn để set crosshair hoặc điều chỉnh tâm ngắm
function setAim(x,y,z){
  console.log(`🎯 AimLock setAim: x=${x.toFixed(6)}, y=${y.toFixed(6)}, z=${z.toFixed(6)}`);
  // VD: gọi API hệ thống native hoặc chỉnh tọa độ tâm ngắm
}

// === Vòng lặp chạy chính ===
(function mainLoop(){
  // Giả lập vị trí người chơi (camera)
  const playerPos = new Vector3(0,0,0);

  // Ví dụ dữ liệu bone head (vị trí lấy từ game hoặc dữ liệu bindpose + position bone head thực tế)
  const boneHeadPos = new Vector3(-0.0456970781, -0.004478302, -0.0200432576);

  // Ví dụ recoil offset (lấy từ game)
  const recoilOffset = new Vector3(0.01, -0.005, 0.002);

  // Khởi tạo engine aimlock (nên tạo bên ngoài để không tạo lại mỗi frame)
  if(!window.aimEngine){
    window.aimEngine = new AimLockHeadLock();
  }

  const now = Date.now();

  // Lấy vị trí aim tính toán
  const aimPos = window.aimEngine.getAimPosition(playerPos, boneHeadPos, recoilOffset, now);

  // Tạo vector ray từ camera tới vị trí aim
  const rayDir = aimPos.subtract(playerPos);

  // Kiểm tra xem có trúng collider bone head không
  const isHitHead = hitDetectColliderBoneHead(playerPos, rayDir, {
    position: boneHeadPos,
    radius: 1.0, // bán kính vùng đầu
    bindpose: {
      e00: -1.34559613e-13, e01: 8.881784e-14, e02: -1.0, e03: 0.487912,
      e10: -2.84512817e-6,  e11: -1.0,        e12: 8.881784e-14, e13: -2.842171e-14,
      e20: -1.0,            e21: 2.84512817e-6, e22: -1.72951931e-13, e23: 0.0,
    }
  });

  if(isHitHead){
    // Nếu trúng đầu, gọi API setAim để lock ngay
    setAim(aimPos.x, aimPos.y, aimPos.z);
    // TODO: Bạn có thể bổ sung auto fire, switch target, ghi log thêm tại đây
  } else {
    // Nếu không trúng có thể reset hoặc làm gì khác (tuỳ mục đích)
    // console.log("❌ Không trúng bone head");
  }

  // Lặp lại mỗi frame ~16ms (60 FPS)
  setTimeout(mainLoop, 16);
})();
