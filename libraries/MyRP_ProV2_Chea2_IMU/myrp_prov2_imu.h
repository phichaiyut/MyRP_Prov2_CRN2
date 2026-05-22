void fw_gyros(int spl, int spr, float kp, float distance_cm, int offset)
{
    if (distance_cm <= 0) {
        Motor(0, 0);
        return;
    }

    int base_speed = min(abs(spl), abs(spr));
    bool is_forward = (spl >= 0 && spr >= 0);

    float kpG = kp;
    float kdG = 0.22;

    float previous_error = 0.0;
    float traveled_distance = 0.0;
    unsigned long last_time = millis();

    // ====================== ค่าที่สามารถปรับได้ ======================
    const float ACCEL_DISTANCE_CM = 20.0;
    const float DECEL_DISTANCE_CM = 25.0;
    const float MIN_SPEED = 10.0;

    // ค่า speed_scale ที่คุณต้องการปรับได้ (ค่าดีฟอลต์ = 0.99)
    float speed_scale = 0.99;        // ← คุณสามารถปรับตรงนี้ได้

    // ตัดสินใจว่าใช้ Ramp หรือไม่
    bool enableRamp = (distance_cm >= 30.0);

    // ถ้าระยะสั้นมาก (< 30) ให้ปรับ speed_scale ได้ง่ายขึ้น
    if (!enableRamp) {
        speed_scale = 1.7;   // คุณสามารถเปลี่ยนเป็น 0.95, 0.98, 1.0 ได้ตามต้องการ
    }

    my.resetAngles();
    float yaw_offset = my.gyro('z');

    while (true)
    {
        
        float yaw = my.gyro('z') - yaw_offset;
        float error = yaw;
        if (error > 180) error -= 360;
        else if (error < -180) error += 360;

        float derivative = error - previous_error;
        float corr = (kpG * error) + (kdG * derivative);

        // คำนวณระยะทาง
        unsigned long current_time = millis();
        float delta_time = (current_time - last_time) / 1000.0;
        traveled_distance += (base_speed * speed_scale) * delta_time;
        last_time = current_time;

        float remaining_cm = distance_cm - traveled_distance;

        if (remaining_cm <= 0.7f) break;

        // ====================== คำนวณ target_speed ======================
        float target_speed = base_speed;

        if (enableRamp)
        {
            if (traveled_distance < ACCEL_DISTANCE_CM) {
                // เร่งช่วงแรก
                target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
            }
            else if (remaining_cm < DECEL_DISTANCE_CM) {
                // ชะลอช่วงสุดท้าย
                target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
            }
        }
        // ถ้า enableRamp = false → ใช้ความเร็วคงที่ตลอดทาง

        // ====================== คำนวณความเร็วซ้าย-ขวา ======================
        int leftSpeed, rightSpeed;
        if (is_forward) {
            leftSpeed  = constrain(target_speed - corr, 0, spl);
            rightSpeed = constrain(target_speed + corr, 0, spr);
        } else {
            leftSpeed  = constrain(-(target_speed - corr), spl, 0);
            rightSpeed = constrain(-(target_speed + corr), spr, 0);
        }

        Motor(leftSpeed, rightSpeed);
        previous_error = error;

        delayMicroseconds(80);
    }

    // ====================== Soft Stop ======================
    if (offset > 0) {
        if (is_forward) {
            Motor(-3, -2); delay(offset);
        } else {
            Motor(2, 3); delay(offset);
        }
        Motor(-1, -1); delay(10);
    } 
    else {
        Motor(0, 0);
    }
    
    Motor(1, 1);      // Pulse เล็กน้อยเพื่อหยุดตรงขึ้น
    delay(5);
}

void bw_gyros(int spl, int spr, float kp, float distance_cm, int offset)
{
    if (distance_cm <= 0) {
        Motor(0, 0);
        return;
    }

    int base_speed = min(abs(spl), abs(spr));
    bool is_backward = true;   // ตรวจว่ากำลังถอยหลัง

    float kpG = kp;
    float kdG = 0.22;

    float previous_error = 0.0;
    float traveled_distance = 0.0;
    unsigned long last_time = millis();

    float speed_scale = 0.99;                    // ค่าเริ่มต้นสำหรับถอยหลัง

    const float ACCEL_DISTANCE_CM = 20.0;
    const float DECEL_DISTANCE_CM = 25.0;
    const float MIN_SPEED = 10.0;

    // ตัดสินใจว่าใช้ Ramp หรือไม่
    bool enableRamp = (distance_cm >= 30.0);

    // ถ้าระยะสั้นมาก (< 30) → ไม่ใช้ Ramp + ปรับ speed_scale
    if (!enableRamp) {
        speed_scale = 1.5;     // คุณสามารถปรับตรงนี้ได้ (แนะนำ 0.92 - 0.97)
    }

    my.resetAngles();
    float yaw_offset = my.gyro('z');

    while (true)
    {
        
        float yaw = my.gyro('z') - yaw_offset;
        float error = yaw;
        if (error > 180) error -= 360;
        else if (error < -180) error += 360;

        float derivative = error - previous_error;
        float corr = (kpG * error) + (kdG * derivative);

        // คำนวณระยะทาง
        unsigned long current_time = millis();
        float delta_time = (current_time - last_time) / 1000.0;
        traveled_distance += (base_speed * speed_scale) * delta_time;
        last_time = current_time;

        float remaining_cm = distance_cm - traveled_distance;

        if (remaining_cm <= 0.8f) break;

        // ====================== คำนวณความเร็ว ======================
        float target_speed = base_speed;

        if (enableRamp)   // ใช้ Ramp เฉพาะระยะยาว (>=30 cm)
        {
            if (traveled_distance < ACCEL_DISTANCE_CM) {
                target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
            }
            else if (remaining_cm < DECEL_DISTANCE_CM) {
                target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
            }
        }
        // ถ้า !enableRamp → ใช้ความเร็วคงที่ตลอดทาง (base_speed)

        int leftSpeed, rightSpeed;

        // ====================== การถอยหลัง ======================
        if (is_backward) {
            leftSpeed  = constrain(-(target_speed + corr), -spl, 0);
            rightSpeed = constrain(-(target_speed - corr), -spr, 0);
        } else {
            leftSpeed  = constrain(target_speed - corr, 0, spl);
            rightSpeed = constrain(target_speed + corr, 0, spr);
        }

        Motor(leftSpeed, rightSpeed);
        previous_error = error;

        delayMicroseconds(80);
    }

    // ====================== Soft Stop ======================
    if (offset > 0) {
        if (is_backward) {
            // ถอยหลัง → เบรกด้วยการเดินหน้าเล็กน้อย
            Motor(4, 4); delay(offset);
        } else {
            // เดินหน้า → เบรกด้วยการถอยหลัง
            Motor(-4, -4); delay(offset);
        }
        Motor(0, 0); delay(10);
    } 
    else {
        Motor(0, 0);
    }

    Motor(1, 1);
    delay(5);
}
