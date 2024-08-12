%%
fig = figure;
dcm_obj = datacursormode(fig);
plot(global_cx, global_cy, '.r')
set(dcm_obj,'UpdateFcn',@myupdatefcn)

%%
figure
plot(global_cyaw, 'ob')