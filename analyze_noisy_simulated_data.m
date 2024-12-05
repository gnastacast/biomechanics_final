addpath MoCapTools/src/
run("model5_params.m")
load('polys.mat')
out = sim('model5_2f.slx');

function rms = calc_rms_error(xa, ta, xb, tb)
    [~, idx] = min(abs(ta - tb'),[],2);
    rms = sqrt( mean((xa - xb(idx,:)).^2));
    rms = mean(rms);
end

%%
noise_levels = linspace(0, 0.1, 11);
rms = zeros(size(noise_levels));
for i = 1:numel(noise_levels)
    for j = 1:10:50
        rng(j)
        xyFP_vec = out.xyFP.Data;
        t = out.xy.Time;
        xy_vec = out.xy.Data;
        noise_mag = noise_levels(i);
        noise = (rand(size(xy_vec)) - 0.5)* noise_mag * 2 .* [1 1];
        xy_vec = xy_vec + noise;
        xyFP_vec = xyFP_vec + noise;
        
        n_legs = 1;
        
        % [vels, liftoff, landing, fp] = analyze_data(t, xy_vec, xyFP_vec, n_legs, ...
        %      0.3, [-inf,2*noise_mag + 0.01], 0.1, 2*noise_mag + 0.01, true);
        
        [vels, liftoff, landing, fp, times] = analyze_data(t, xy_vec, xyFP_vec, n_legs, ...
             1.2, [inf,0.05], 0.05, false);
        % hold on
        % plot(t, out.in_stance.Data)
        % ylim([0,1.5])
        % leg_lengths = vecnorm(fp,2,2)
        
        
        load('polys.mat')
        run("model5_params.m")
        
        % figure(1000)
        out2 = sim("model5_controlled.slx");
    
        rms(i) = rms(i) + calc_rms_error(out.xy.Data, out.xy.Time, out2.xy.Data, out2.xy.Time) / 3;
    end
    if i == 2 || i == numel(noise_levels)
        figure(i);
        clf(i);
        hold on
        plot(xy_vec(:,1),xy_vec(:,2), "LineWidth",1)
        blanks= [];
        for k = 1:20:size(out.xyFP.Data)
             plot([xy_vec(k, 1), xyFP_vec(k, 1)], ...
                  [xy_vec(k, 2), xyFP_vec(k,2)], 'Color',[.7,.7,.7])
             blanks = [blanks, ""];
        end
        plot(out2.xy.Data(:,1), out2.xy.Data(:,2), "LineWidth",2)
        scatter(landing(:,1), landing(:,2))
        ends = fp + landing;
        scatter(ends(:,1), ends(:,2))
        plot([0,10],[0,0], 'k')
        for k = 1:size(ends,1)
            plot([landing(k, 1), ends(k, 1)], ...
                 [landing(k, 2), ends(k,2)], 'k', "LineWidth",2)
        end
        axis equal
        ylim([0,3])
        xlim([0,10])
        title("Simulated Noise = " + noise_mag)
        legend(["Original CoM", blanks, "Replica CoM"])
    end
end
%%
figure(1000)
clf(1000)
plot(noise_levels, rms)
xlabel("Noise magnitude (m)")
ylabel("RMS error (m)")

disp(rms)