xSeq = simRes.logsout.getElement('xSeq').Values;
mvSeq = simRes.logsout.getElement('mvSeq').Values;
yrefSeq = simRes.logsout.getElement('seq').Values;

time = 2.5;
xk = xSeq.getsampleusingtime(time);
xk = xk.Data(1, :);
mv = mvSeq.getsampleusingtime(time);
mv = mv.Data(1, :);
yref = yrefSeq.getsampleusingtime(time).Data;

[mv,nloptions,info] = nlmpcmove(nlmpc_ee,xk,mv,yref);