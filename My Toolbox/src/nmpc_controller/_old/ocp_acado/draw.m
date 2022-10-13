figure;

subplot(2,2,1)
plot(out.STATES(:,1), out.STATES(:,2), 'r')
title('SC X');

subplot(2,2,2)
plot(out.STATES(:,1), out.STATES(:,3), 'r')
title('SC Y');

subplot(2,2,3)
plot(out.STATES(:,1), out.STATES(:,8), 'r')
title('Joint 1');

subplot(2,2,4)
plot(out.STATES(:,1), out.STATES(:,9), 'r')
title('Joint 2');

figure;
subplot(2,2,1)
plot(out.CONTROLS(:,1), out.CONTROLS(:,2), 'r')
title('Tau x');

subplot(2,2,2)
plot(out.CONTROLS(:,1), out.CONTROLS(:,3), 'r')
title('Tau y');

subplot(2,2,3)
plot(out.CONTROLS(:,1), out.CONTROLS(:,8), 'r')
title('Tau qm1');

subplot(2,2,4)
plot(out.CONTROLS(:,1), out.CONTROLS(:,9), 'r')
title('Tau qm2');