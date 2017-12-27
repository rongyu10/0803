function idx_sample = my_randsample(ndata, nsample)

tmp_data = rand(ndata,1);

[~,I] = sort(tmp_data);
idx_sample = I(1:nsample);

end
