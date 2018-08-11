function fc_mdl = fc_estimation(input_variable,speed_limit, std_input_param, gmm_cell,spd_limit_list)

fuel_idle = 0.000172399445772618;
fc_mdl_list = zeros(size(input_variable,1),1);
input_test_cell = cell(1,size(input_variable,2));
for input_id = 1:size(input_variable,2)
    input_test_cell{input_id} = repmat(input_variable(:,input_id),1,size(input_variable,2)+1-input_id)...
        .*input_variable(:,input_id:end);
end
input_test_mix = horzcat(input_test_cell{:});
input_test_mix = [input_test_mix,input_variable];
% spd_limit_id = find(round(mps2mph(spd_limit_list)) == round(mps2mph(speed_limit)));
[~,spd_limit_id_list] = ismember(round(mps2mph(speed_limit)),round(mps2mph(spd_limit_list)));
%     chg_spd_bd = prctile(inputTestMdl_spd_limit(:,12),[0.05,99.95]);
%     filter_chgSpd = inputTestMdl_spd_limit(:,12)>chg_spd_bd(1) & inputTestMdl_spd_limit(:,12)<chg_spd_bd(2);

%     inputTestMdl_spd_limit = inputTestMdl_spd_limit(filter_chgSpd,:);
%     fc_test_spd_limit = fc_test_spd_limit(filter_chgSpd,:);
spd_limit_id_list_uniq = unique(spd_limit_id_list);
for spd_limit_list_id = 1:length(spd_limit_id_list_uniq)
    spd_limit_id = spd_limit_id_list_uniq(spd_limit_list_id);
    input_test_mix_spd_limit = input_test_mix(spd_limit_id_list==spd_limit_id,:);
    inputTestMdl_stan = (input_test_mix_spd_limit-repmat(std_input_param{spd_limit_id,1},size(input_test_mix_spd_limit,1),1))./repmat(std_input_param{spd_limit_id,2},size(input_test_mix_spd_limit,1),1);
%     layer1_unsup_gmm_vb = gmdistribution(gmm_cell{spd_limit_id,3}.mu(1:end,1:end-1),...
%                                     gmm_cell{spd_limit_id,3}.Sigma(1:end-1,1:end-1,:),...
%                                     gmm_cell{spd_limit_id,3}.ComponentProportion);
%     [~,~,layer1_posterior] = cluster(layer1_unsup_gmm_vb,inputTestMdl_stan);
    [~,~,layer1_posterior] = cluster(gmm_cell{spd_limit_id,2},inputTestMdl_stan);
    dbstop if error

    conditional_fc = zeros(size(inputTestMdl_stan,1),1);
    for component_id = 1:gmm_cell{spd_limit_id,2}.NumComponents
        component_mu = gmm_cell{spd_limit_id,1}.mu(component_id,:);
        component_sigma = gmm_cell{spd_limit_id,1}.Sigma(:,:,component_id);
        conditional_mu = component_mu(end)+component_sigma(end,1:end-1)*(component_sigma(1:end-1,1:end-1)\(inputTestMdl_stan-repmat(component_mu(1:end-1),size(inputTestMdl_stan,1),1))');
        conditional_fc = conditional_fc+ conditional_mu'.*layer1_posterior(:,component_id);
    end

    conditional_fc = (conditional_fc*std_input_param{spd_limit_id,4}+std_input_param{spd_limit_id,3});
    fc_mdl_list(spd_limit_id_list==spd_limit_id) = conditional_fc;
end
tt_spd_limit = input_test_mix(:,14)./input_test_mix(:,11);
fc_min_spd_limit = fuel_idle*tt_spd_limit;
fc_mdl = max(fc_min_spd_limit,fc_mdl_list);


