classdef CommAgent < handle
%COMMAGENT  Simulates acoustic modem uplink to surface vessel
%  Applies latency jitter and packet loss typical of underwater comms.

    properties
        latency_mean = 0.3      % mean latency [s]
        latency_std  = 0.1      % std deviation [s]
        packet_loss  = 0.05     % 5% packet loss probability
        log_interval = 5.0      % console log every N seconds
        last_log     = -999
        tx_count     = 0
        loss_count   = 0
    end

    methods
        function obj = CommAgent()
            fprintf('[CommAgent] Acoustic modem initialised.\n');
            fprintf('           Latency: %.0fms±%.0fms | Loss: %.0f%%\n', ...
                    obj.latency_mean*1000, obj.latency_std*1000, ...
                    obj.packet_loss*100);
        end

        function msg = broadcast(obj, pos, target, nav_str, fault_str, t)
            obj.tx_count = obj.tx_count + 1;
            dropped = rand() < obj.packet_loss;
            if dropped
                obj.loss_count = obj.loss_count + 1;
                msg = '[COMM] *** PACKET LOST ***';
                return;
            end
            lat = obj.latency_mean + obj.latency_std*randn();
            lat = max(0.05, lat);
            msg = sprintf('[COMM] t=%.1fs | pos=(%.1f,%.1f,%.1f) | lat=%.2fs', ...
                          t, pos(1), pos(2), pos(3), lat);
            if t - obj.last_log >= obj.log_interval
                obj.last_log = t;
                pdr = 100*(1 - obj.loss_count/max(1,obj.tx_count));
                fprintf('[CommAgent] PDR=%.1f%% | %s | %s\n', ...
                        pdr, nav_str(1:min(30,end)), fault_str);
            end
        end
    end
end
