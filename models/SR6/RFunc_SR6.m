function [Rb,Ra,Rm,Tee] = RFunc_SR6(in1)
%RFunc_SR6
%    [Rb,Ra,Rm,Tee] = RFunc_SR6(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    14-Nov-2022 09:02:24

Rx = in1(1,:);
Ry = in1(2,:);
Rz = in1(3,:);
p = in1(5,:);
qm1 = in1(7,:);
qm2 = in1(8,:);
qm3 = in1(9,:);
qm4 = in1(10,:);
qm5 = in1(11,:);
qm6 = in1(12,:);
r = in1(4,:);
y = in1(6,:);
t2 = cos(p);
t3 = cos(qm1);
t4 = cos(qm2);
t5 = cos(qm3);
t6 = cos(qm4);
t7 = cos(qm5);
t8 = cos(qm6);
t9 = cos(r);
t10 = cos(y);
t11 = sin(p);
t12 = sin(qm1);
t13 = sin(qm2);
t14 = sin(qm3);
t15 = sin(qm4);
t16 = sin(qm5);
t17 = sin(qm6);
t18 = sin(r);
t19 = sin(y);
t20 = t9.*t10;
t21 = t3.*t11;
t22 = t9.*t19;
t23 = t10.*t18;
t24 = t11.*t12;
t25 = t18.*t19;
t26 = -t16;
t27 = -t17;
t28 = t2.*t12.*t19;
t32 = t2.*t4.*t9;
t33 = t2.*t3.*t10;
t34 = t2.*t3.*t18;
t35 = t2.*t9.*t13;
t36 = t2.*t3.*t19;
t37 = t2.*t10.*t12;
t39 = t2.*t12.*t18;
t72 = t8.*6.123233995736766e-17;
t73 = t17.*6.123233995736766e-17;
t75 = t2.*t9.*6.123233995736766e-17;
t90 = t8.*3.749399456654644e-33;
t91 = t17.*3.749399456654644e-33;
t111 = t2.*t9.*3.749399456654644e-33;
t29 = t11.*t22;
t30 = t11.*t23;
t31 = t11.*t25;
t38 = t11.*t20;
t40 = -t28;
t43 = -t33;
t44 = -t39;
t45 = t24+t34;
t74 = -t73;
t76 = t23.*6.123233995736766e-17;
t77 = t24.*6.123233995736766e-17;
t78 = t25.*6.123233995736766e-17;
t79 = -t75;
t80 = t34.*6.123233995736766e-17;
t81 = t37.*6.123233995736766e-17;
t83 = t28.*6.123233995736766e-17;
t94 = t8+t91;
t97 = t27+t90;
t112 = t23.*3.749399456654644e-33;
t113 = t25.*3.749399456654644e-33;
t114 = -t111;
t118 = t72+t73;
t41 = -t29;
t42 = -t30;
t46 = t20+t31;
t47 = t25+t38;
Rb = reshape([t2.*t10,t2.*t19,-t11,-t22+t30,t46,t2.*t18,t47,-t23+t29,t2.*t9],[3,3]);
if nargout > 1
    Ra = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0],[6,6]);
end
if nargout > 2
    t48 = t21+t44;
    t82 = t38.*6.123233995736766e-17;
    t84 = t29.*6.123233995736766e-17;
    t85 = -t83;
    t87 = t4.*t45.*6.123233995736766e-17;
    t88 = t13.*t45.*6.123233995736766e-17;
    t93 = t45+t79;
    t115 = t38.*3.749399456654644e-33;
    t116 = t29.*3.749399456654644e-33;
    t119 = t72+t74;
    mt1 = [t3,t12,0.0,-t12,t3,0.0,0.0,0.0,1.0,t4,t13.*6.123233995736766e-17,t13,-t13,t4.*6.123233995736766e-17,t4,0.0,-1.0,6.123233995736766e-17,t5,t14,0.0,-t14,t5,0.0,0.0,0.0,1.0,t6,t15,0.0,-t15,t6,0.0,0.0,0.0,1.0];
    mt2 = [t7,t16.*6.123233995736766e-17,t26,t26,t7.*6.123233995736766e-17,-t7,0.0,1.0,6.123233995736766e-17,t119,t94,t17,-t72+t74,t97,t8,1.0];
    mt3 = [-6.123233995736766e-17,6.123233995736766e-17,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0];
    Rm = reshape([mt1,mt2,mt3],3,21);
end
if nargout > 3
    t49 = t22+t42;
    t50 = t23+t41;
    t51 = t3.*t46;
    t52 = t4.*t47;
    t53 = t12.*t46;
    t54 = t13.*t47;
    t55 = t4.*t48;
    t56 = t13.*t48;
    t86 = -t84;
    t95 = t7.*t93;
    t96 = t16.*t93;
    t117 = -t116;
    t57 = t3.*t49;
    t58 = t4.*t50;
    t59 = t12.*t49;
    t60 = t13.*t50;
    t61 = -t54;
    t62 = -t55;
    t64 = t36+t53;
    t66 = t40+t51;
    t89 = t51.*6.123233995736766e-17;
    t99 = t4.*(t28-t51).*(-6.123233995736766e-17);
    t101 = t13.*(t28-t51).*(-6.123233995736766e-17);
    t103 = t4.*(t28-t51).*6.123233995736766e-17;
    t104 = t32+t56+t87;
    t63 = -t60;
    t65 = t37+t57;
    t67 = t4.*t64;
    t68 = t13.*t64;
    t69 = t43+t59;
    t70 = -t4.*(t33-t59);
    t71 = -t13.*(t33-t59);
    t92 = t57.*6.123233995736766e-17;
    t105 = t35+t62+t88;
    t106 = t5.*t104;
    t107 = t14.*t104;
    t135 = t66+t76+t86;
    t98 = t4.*t65.*6.123233995736766e-17;
    t100 = t13.*t65.*6.123233995736766e-17;
    t108 = t5.*t105;
    t109 = t14.*t105;
    t120 = t58+t68+t103;
    t121 = t63+t67+t101;
    t126 = -t5.*(t60-t67+t13.*(t28-t51).*6.123233995736766e-17);
    t127 = -t14.*(t60-t67+t13.*(t28-t51).*6.123233995736766e-17);
    t132 = t5.*(t60-t67+t13.*(t28-t51).*6.123233995736766e-17);
    t134 = t65+t78+t82;
    t138 = t7.*t135;
    t139 = t16.*t135;
    t141 = t26.*t135;
    t102 = -t98;
    t110 = -t109;
    t123 = t5.*t120;
    t124 = t14.*t120;
    t125 = t61+t70+t100;
    t128 = -t5.*(-t52+t98+t13.*(t33-t59));
    t129 = -t14.*(-t52+t98+t13.*(t33-t59));
    t130 = -t5.*(t54-t100+t4.*(t33-t59));
    t131 = -t14.*(t54-t100+t4.*(t33-t59));
    t133 = t5.*(t54-t100+t4.*(t33-t59));
    t136 = t7.*t134;
    t137 = t16.*t134;
    t140 = t26.*t134;
    t142 = t107+t108;
    t159 = -t6.*(t5.*(-t52+t98+t13.*(t33-t59))+t14.*(t54-t100+t4.*(t33-t59)));
    t160 = -t15.*(t5.*(-t52+t98+t13.*(t33-t59))+t14.*(t54-t100+t4.*(t33-t59)));
    t122 = t52+t71+t102;
    t143 = t106+t110;
    t144 = t6.*t142;
    t145 = t15.*t142;
    t150 = t123+t127;
    t151 = t124+t132;
    t154 = t128+t131;
    t157 = t129+t133;
    t146 = t6.*t143;
    t147 = t15.*t143;
    t148 = -t145;
    t152 = t6.*t150;
    t153 = t15.*t150;
    t155 = t6.*t151;
    t156 = t15.*t151;
    t161 = t6.*t157;
    t162 = t15.*t157;
    t149 = -t146;
    t158 = -t156;
    t163 = -t162;
    t164 = t144+t147;
    t165 = t146+t148;
    t171 = t153+t155;
    t175 = t160+t161;
    t177 = t7.*(t161-t15.*(t5.*(-t52+t98+t13.*(t33-t59))+t14.*(t54-t100+t4.*(t33-t59))));
    t178 = t16.*(t161-t15.*(t5.*(-t52+t98+t13.*(t33-t59))+t14.*(t54-t100+t4.*(t33-t59))));
    t182 = t7.*(t162+t6.*(t5.*(-t52+t98+t13.*(t33-t59))+t14.*(t54-t100+t4.*(t33-t59)))).*(-6.123233995736766e-17);
    t183 = t16.*(t162+t6.*(t5.*(-t52+t98+t13.*(t33-t59))+t14.*(t54-t100+t4.*(t33-t59)))).*(-6.123233995736766e-17);
    t184 = t7.*(t162+t6.*(t5.*(-t52+t98+t13.*(t33-t59))+t14.*(t54-t100+t4.*(t33-t59)))).*6.123233995736766e-17;
    t166 = t7.*t164;
    t167 = t16.*t164;
    t168 = t26.*t164;
    t169 = t7.*(t145+t149).*(-6.123233995736766e-17);
    t170 = t16.*(t145+t149).*(-6.123233995736766e-17);
    t172 = t152+t158;
    t173 = t7.*t171;
    t174 = t16.*t171;
    t176 = t159+t163;
    t185 = t77+t80+t114+t145+t149;
    t192 = t136+t178+t184;
    t193 = t140+t177+t183;
    t179 = t7.*t172.*6.123233995736766e-17;
    t180 = t16.*t172.*6.123233995736766e-17;
    t186 = t85+t89+t112+t117+t172;
    t187 = t81+t92+t113+t115+t176;
    t188 = t96+t166+t170;
    t189 = t95+t168+t169;
    t181 = -t179;
    t191 = t141+t173+t180;
    t190 = t138+t174+t181;
    et1 = t25.*2.295845021658468e-49;
    et2 = t37.*3.749399456654644e-33;
    et3 = t38.*2.295845021658468e-49;
    et4 = t57.*3.749399456654644e-33+t136.*6.123233995736766e-17;
    et5 = t162.*(-6.123233995736766e-17)+t178.*6.123233995736766e-17+t193-t6.*(t5.*(-t52+t98+t13.*(t33-t59))+t14.*(t54-t100+t4.*(t33-t59))).*6.123233995736766e-17;
    et6 = t7.*(t162+t6.*(t5.*(-t52+t98+t13.*(t33-t59))+t14.*(t54-t100+t4.*(t33-t59)))).*3.749399456654644e-33;
    et7 = Rx+t25.*8.1e-1;
    et8 = t33./1.0e+1+t37.*3.0e-1;
    et9 = t38.*8.1e-1;
    et10 = t54.*(2.61e+2./1.0e+2)+t57.*3.0e-1-t59./1.0e+1+t133.*(2.61e+2./1.0e+2);
    et11 = t136.*(-9.999999999999997e-2)-t137.*5.0e-1+t161./1.0e+1;
    et12 = t162.*(-1.4e-1)+t177.*5.0e-1;
    et13 = t178.*(-9.999999999999997e-2)-t6.*(t5.*(-t52+t98+t13.*(t33-t59))+t14.*(t54-t100+t4.*(t33-t59))).*1.4e-1;
    et14 = t15.*(t5.*(-t52+t98+t13.*(t33-t59))+t14.*(t54-t100+t4.*(t33-t59))).*(-1.0./1.0e+1)-t13.*t65.*1.598164072887296e-16+t4.*(t33-t59).*(2.61e+2./1.0e+2);
    et15 = t7.*(t162+t6.*(t5.*(-t52+t98+t13.*(t33-t59))+t14.*(t54-t100+t4.*(t33-t59)))).*(-6.123233995736764e-18);
    et16 = t16.*(t162+t6.*(t5.*(-t52+t98+t13.*(t33-t59))+t14.*(t54-t100+t4.*(t33-t59)))).*(-3.061616997868383e-17)-t14.*(-t52+t98+t13.*(t33-t59)).*(2.61e+2./1.0e+2);
    et17 = t23.*(-2.295845021658468e-49);
    et18 = t28.*3.749399456654644e-33;
    et19 = t29.*2.295845021658468e-49;
    et20 = t51.*(-3.749399456654644e-33)-t138.*6.123233995736766e-17+t139;
    et21 = t152.*(-6.123233995736766e-17)+t156.*6.123233995736766e-17-t173-t174.*6.123233995736766e-17-t180;
    et22 = t7.*t172.*3.749399456654644e-33;
    et23 = Ry-t23.*8.1e-1;
    et24 = t28.*3.0e-1;
    et25 = t29.*8.1e-1;
    et26 = t36./1.0e+1-t51.*3.0e-1+t53./1.0e+1-t60.*(2.61e+2./1.0e+2)+t67.*(2.61e+2./1.0e+2)-t124.*(2.61e+2./1.0e+2)-t132.*(2.61e+2./1.0e+2);
    et27 = t138.*9.999999999999997e-2+t139.*5.0e-1;
    et28 = t152.*(-1.4e-1)-t153./1.0e+1-t155./1.0e+1+t156.*1.4e-1;
    et29 = t173.*(-5.0e-1)+t174.*9.999999999999997e-2;
    et30 = t7.*t172.*(-6.123233995736764e-18);
    et31 = t16.*t172.*(-3.061616997868383e-17)-t13.*(t28-t51).*1.598164072887296e-16;
    et32 = t24.*(-3.749399456654644e-33);
    et33 = t34.*(-3.749399456654644e-33)-t95.*6.123233995736766e-17;
    et34 = t145.*(-6.123233995736766e-17)+t146.*6.123233995736766e-17+t167.*6.123233995736766e-17+t188;
    et35 = t7.*(t145+t149).*3.749399456654644e-33;
    et36 = t2.*t9.*2.295845021658468e-49;
    et37 = Rz-t21./1.0e+1-t24.*3.0e-1;
    et38 = t34.*(-3.0e-1)+t35.*(2.61e+2./1.0e+2)+t39./1.0e+1-t55.*(2.61e+2./1.0e+2);
    et39 = t95.*9.999999999999997e-2+t96.*5.0e-1+t107.*(2.61e+2./1.0e+2)+t108.*(2.61e+2./1.0e+2)+t144./1.0e+1;
    et40 = t145.*(-1.4e-1)+t146.*1.4e-1+t147./1.0e+1;
    et41 = t166.*5.0e-1-t167.*9.999999999999997e-2;
    et42 = t7.*(t145+t149).*(-6.123233995736764e-18);
    et43 = t16.*(t145+t149).*(-3.061616997868383e-17);
    et44 = t2.*t9.*8.1e-1;
    et45 = t13.*t45.*1.598164072887296e-16;
    Tee = reshape([t17.*t187-t94.*t192+t119.*t193,-t27.*(t83-t89-t112+t116-t152+t156)+t94.*t190-t119.*t191,t27.*t185+t94.*t189+t119.*t188,0.0,t8.*t187-t97.*t192-t118.*t193,t8.*(t83-t89-t112+t116-t152+t156)+t97.*t190+t118.*t191,-t8.*t185+t97.*t189-t118.*t188,0.0,et1+et2+et3+et4+et5+et6,et17+et18+et19+et20+et21+et22,et32+et33+et34+et35+et36,0.0,et7+et8+et9+et10+et11+et12+et13+et14+et15+et16,et23+et24+et25+et26+et27+et28+et29+et30+et31,et37+et38+et39+et40+et41+et42+et43+et44+et45,1.0],[4,4]);
end
