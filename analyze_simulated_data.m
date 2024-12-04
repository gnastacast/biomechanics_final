addpath MoCapTools/src/
run("model5_params.m")
load('polys.mat')
out = sim('model5_2f.slx');

%%



xyFP_vec = out.xyFP.Data;
t = out.xy.Time;
xy_vec = out.xy.Data;
noise_mag = 0.00;
noise = rand(size(xy_vec)) * noise_mag .* [1 1];
xy_vec = xy_vec + noise;
xyFP_vec = xyFP_vec + noise;

n_legs = 1;

[vels, liftoff, landing, fp] = analyze_data(t, xy_vec, xyFP_vec, n_legs, ...
     0.02, [-inf,0.05], 0.01, 0.01, true);
hold on
plot(t, out.in_stance.Data)
leg_lengths = vecnorm(fp,2,2)


load('polys.mat')
run("model5_params.m")

out2 = sim("model5_controlled.slx");

figure;

hold on
plot(out.xy.Data(:,1), out.xy.Data(:,2), "LineWidth",2)
blanks= [];
for i = 1:20:size(out.xyFP.Data)
     plot([out.xy.Data(i, 1), out.xyFP.Data(i, 1)], ...
          [out.xy.Data(i, 2), out.xyFP.Data(i,2)], 'Color',[.7,.7,.7])
     blanks = [blanks, ""];
end
plot(out2.xy.Data(:,1), out2.xy.Data(:,2), "LineWidth",2)
scatter(landing(:,1), landing(:,2))
ends = fp + landing;
scatter(ends(:,1), ends(:,2))
plot([0,10],[0,0], 'k')
for i = 1:size(ends,1)
    plot([landing(i, 1), ends(i, 1)], ...
         [landing(i, 2), ends(i,2)], 'k', "LineWidth",2)
end
axis equal
ylim([0,3])
xlim([0,10])
title("Results using foot placement controller")
legend(["Original CoM", blanks, "Replica CoM"])