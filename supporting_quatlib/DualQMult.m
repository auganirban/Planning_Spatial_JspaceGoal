function [dualQuaternion] = DualQMult(q1,q2)
    %Dual Quaternion Multiplication:
    q1_r = q1(1,1:4);
    q1_d = q1(1,5:8);

    q2_r = q2(1,1:4);
    q2_d = q2(1,5:8);

    real = QMult(q1_r,q2_r);
    dual1 = QMult(q1_r,q2_d);
    dual2 = QMult(q1_d,q2_r);

    dual = dual1 + dual2;
    dualQuaternion = [real dual];
end
