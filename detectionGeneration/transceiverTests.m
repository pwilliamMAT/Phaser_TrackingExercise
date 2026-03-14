%% Save data from the output of the transceiver

tx = TransceiverDetectionGenerator();
data = tx.receiveIQ(-15:15:15);

%% Range Doppler

tl = tiledlayout(figure,"vertical");

% Create range-Doppler response
fs = tx.Radar.Waveform.SampleRate;
fc = tx.Radar.TransmitAntenna.OperatingFrequency;
prf = tx.Radar.Waveform.PRF;
mf = tx.Radar.Waveform.getMatchedFilter();
rdresp = phased.RangeDopplerResponse(SampleRate=fs,DopplerOutput="Speed",PRFSource="Property",PRF=prf,OperatingFrequency=fc);

for id = 1:length(data)
    % Get data
    d = squeeze(sum(data{id}.IQ,2));
    pointdir = data{id}.PointDir;

    % Process
    [resp,range,dop] = rdresp(d,mf);

    % Plot
    ax = nexttile(tl);
    keeprange = range >= 0 & range <= 50;
    imagesc(ax,dop,range(keeprange),abs(resp(keeprange,:)));
    title(ax,['Point Direction = ',num2str(pointdir)]);
end

%% Dechirp

p1 = data{1}.IQ;
mf1 = tx.Radar.Waveform.getMatchedFilter();
rr = phased.RangeResponse(SampleRate=tx.Radar.Waveform.SampleRate);
[resp,range] = rr(p1(:,1,1),mf1);
keeprange = range >= 0 & range <= 50;
plot(range(keeprange),abs(resp(keeprange)));