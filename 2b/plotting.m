figure;

subplot(3, 1, 1);
plot(cumsum(ZYXsdeg(1, :)));
xlabel('Image Number');
ylabel('Accumulated Roll (deg)');
grid on;

subplot(3, 1, 2);
plot(cumsum(ZYXsdeg(2, :)));
xlabel('Image Number');
ylabel('Accumulated Yaw (deg)');
grid on;

subplot(3, 1, 3);
plot(cumsum(ZYXsdeg(3, :)));
xlabel('Image Number');
ylabel('Accumulated Pitch (deg)');
grid on;
