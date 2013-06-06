%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lixd add 08272011
% Function name: getMotorEff
% input paramter: v    vehicle speed (m/s)
%                 T    vehicle torque (Nm)
%                 r    vehicle tire radius (m)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function eff_final = getMotorEff( v, T, r )

% from Yan
%---------------------------------------------------------------------------------------------
    %% driving_efficiency
    % clear,clc
    W_rpm=50.45:50.45:771.75;
    Tx=0:4:104;
    eff_dri1=[
        0         0         0         0         0         0         0         0         0         0         0
        0.0823    0.1768    0.2761    0.4332    0.5153    0.6133    0.6550    0.6009    0.6252    0.6291    0.6491
        0.1647    0.3524    0.5274    0.8245    0.8196    0.8366    0.8199    0.8089    0.8304    0.8491    0.8956
        0.2470    0.5271    0.7069    0.8837    0.8932    0.9104    0.8955    0.9149    0.9676    0.9617    0.9354
        0.3294    0.6295    0.7828    0.9191    0.9354    0.9134    0.9391    0.9827    0.9745    0.9292    0.9268
        0.4053    0.6968    0.8369    0.9419    0.9629    0.9467    0.9444    0.9704    0.9614    0.9296    0.9222
        0.4773    0.7504    0.8774    0.9579    0.9745    0.9261    0.9068    0.9305    0.9532    0.9302    0.9197
        0.5468    0.7941    0.8433    0.9076    0.9202    0.9121    0.9071    0.9302    0.9477    0.9310    0.9218
        0.5777    0.7501    0.7853    0.8735    0.9042    0.9022    0.9075    0.8965    0.9118    0.9059    0.9041
        0.5411    0.7095    0.7655    0.8488    0.8860    0.8775    0.8749    0.8634    0.8824    0.8876    0.8912
        0.5277    0.6801    0.7332    0.8148    0.8455    0.8342    0.8359    0.8388    0.8434    0.8738    0.8797
        0.5173    0.6498    0.7045    0.7665    0.8153    0.8168    0.8292    0.8226    0.8265    0.8528    0.8481
        0.4950    0.6333    0.6825    0.7440    0.7777    0.7886    0.8111    0.8107    0.8113    0.8319    0.8240
        0.4700    0.6200    0.6763    0.7383    0.7718    0.7772    0.7916    0.8010    0.7991    0.8153    0.8101
        0.4505    0.5985    0.6605    0.7226    0.7552    0.7678    0.7759    0.7891    0.7896    0.8050    0.8148
        0.4350    0.5862    0.6474    0.7096    0.7416    0.7605    0.7591    0.7800    0.7836    0.7929    0.8029
        0.4133    0.5659    0.6364    0.7078    0.7328    0.7471    0.7503    0.7723    0.7787    0.7912    0.7951
        0.4020    0.5572    0.6267    0.6888    0.7279    0.7358    0.7429    0.7659    0.7747    0.7904    0.7885
        0.3865    0.5427    0.6167    0.6803    0.7237    0.7249    0.7460    0.7538    0.7720    0.7849    0.7783
        0.3789    0.5318    0.6015    0.6729    0.7125    0.7207    0.7394    0.7506    0.7629    0.7803    0.7757
        0.3613    0.5225    0.5945    0.6601    0.7012    0.7171    0.7334    0.7412    0.7550    0.7833    0.7717
        0.3511    0.5144    0.5849    0.6490    0.6914    0.7142    0.7202    0.7419    0.7616    0.7686    0.7642
        0.3384    0.4981    0.5717    0.6393    0.6836    0.7014    0.7159    0.7427    0.7640    0.7557    0.7635
        0.3249    0.4842    0.5647    0.6295    0.6725    0.6954    0.7176       NaN       NaN       NaN       NaN
        0.3136    0.4723    0.5545    0.6171    0.6629    0.6884       NaN       NaN       NaN       NaN       NaN
        0.3045    0.4588    0.5454    0.6081    0.6550    0.6777       NaN       NaN       NaN       NaN       NaN
           NaN       NaN       NaN       NaN       NaN       NaN       NaN       NaN       NaN       NaN       NaN];
    eff_dri2=[   
        0         0         0         0
        0.6632    0.6316    0.6041    0.5930
        0.8933    0.8002    0.7931    0.7965
        0.9190    0.8891    0.8431    0.8578
        0.8906    0.9048    0.9052    0.9267
        0.9074    0.9451    0.9309    0.9314
        0.9134    0.9209    0.9288    0.9352
        0.9105    0.9256    0.9019    0.9384
        0.8864    0.9295    0.9048    0.9203
        0.8880    0.9166    0.8908    0.9142
        0.8786    0.8617    0.8628    0.8944
        0.8654    0.8440    0.8434    0.8766
        0.8408    0.8188    0.8313    0.8490
        0.8102    0.8195    0.8099    0.8274
        0.8022    0.8061    0.8026    0.8208
        0.7957    0.8058    0.7941    0.8151
        0.7903    0.8058    0.7965    0.8014
        0.7847    0.7980    0.7900    0.7983
        0.7878    0.7999    0.7865    0.8010
        0.7831    0.7938    0.7846    0.7957
        0.7796    0.7890    0.7830    0.7912
        0.7713    0.7786    0.7830    0.7881
        0.7701    0.7758    0.7744    0.7861
           NaN       NaN    0.7670    0.7845
           NaN       NaN       NaN       NaN
           NaN       NaN       NaN       NaN
           NaN       NaN       NaN       NaN];
    eff_dri=[eff_dri1,eff_dri2];
%     mesh(W_rpm, Tx, eff_dri);

    %% braking_efficiency
    Tb =[0 6 12 18 24 30 36 42 48 54 60 66 72 78 82];
    Spd_rpm =[200 250 300 350 400 450 500 550 600 650 700];

    Power =[
       2.3397    2.4136    3.1605    2.4190    3.3305    2.9097    3.2744    2.2726    2.5796    2.8139     8.4930      %0 Nm
       1.4165    2.5798    3.5770    6.8345    7.0947    10.799    10.196    24.134    33.207    53.456     97.733      %6
       1.9965    5.0214    11.8601   29.804    53.8481   92.462    113.169   130.333   166.812   235.345    302.745     %12
       7.7931    28.2311   73.0387   125.694   182.681   241.179   276.110   316.464   388.349   473.254    569.034     %18
       37.5257   99.4504   175.775   270.377   331.631   431.482   511.008   576.491   655.339   766.991    885.165     %24
       86.0749   186.262   291.618   412.051   521.270   618.988   743.634   813.542   929.501   1058.83    1195.52     %30
       147.110   269.813   408.104   552.723   682.092   827.718   934.728   1042.96   1196.22   1346.34    1493.08     %36
       204.473   353.129   518.677   685.576   841.699   1010.67   1135.41   1278.73   1466.60   1623.73    1782.86     %42
       257.381   437.097   630.012   816.119   1003.04   1186.81   1369.59   1530.68   1719.76   1885.47    2065.78     %48
       303.799   510.412   731.393   933.448   1151.33   1354.78   1561.80   1748.06   1965.89   2143.27    2329.96     %54
       358.894   577.299   818.163   1055.51   1245.50   1514.73   1741.37   1889.91   2194.87   2390.63    2587.21     %60
       404.992   640.309   906.198   1165.42   1328.86   1665.38   1921.18   2166.55   2420.65   2619.32    2825.43     %66
       447.844   700.507   995.382   1277.90   1529.90   1820.12   2083.66   2363.21   2649.53   2858.57    3033.20     %72
%%%% temp
       447.844   700.507   995.382   1277.90   1529.90   1820.12   2083.66   2363.21   2649.53   2858.57    3033.20     %72
       447.844   700.507   995.382   1277.90   1529.90   1820.12   2083.66   2363.21   2649.53   2858.57    3033.20     %72
       ];

    Eff=Power./(Tb'*Spd_rpm*pi/30);
    Eff(1,:)=0;
   

%     figure('color',[1 1 1]);
%     axes1 = axes('FontWeight','bold','FontSize',14,...
%         'FontName','Times New Roman');
%     % box(axes1,'on');
%     grid(axes1,'on');
%     % hold(axes1,'all');
%     mesh(Spd_rpm, Tb, Eff);
%     xlabel('Wheel Speed [rpm]','FontWeight','bold','FontSize',12,...
%         'FontName','Times New Roman');
%     ylabel('Braking Torque [Nm]','FontWeight','bold','FontSize',12,...
%         'FontName','Times New Roman');
%     zlabel('Effiency','FontWeight','bold','FontSize',12,...
%         'FontName','Times New Roman');
%---------------------------------------------------------------------------------------------




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lixd add 08272011
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % driving or regeneration
    if T<0;
        curr_RotSpd = 60*v/(2*pi*r);
        T = abs(T);
        % get the two endpoints of the rotational speed interval
        for i =1:length(Spd_rpm);
            if curr_RotSpd < Spd_rpm(i);
                if i ==1;
                    pre_RotSpd = 0;
                    next_RotSpd = Spd_rpm(i);
                    pre_Clm = NaN;
                    next_Clm = i;
                else
                    pre_RotSpd = Spd_rpm(i-1);
                    next_RotSpd = Spd_rpm(i);
                    pre_Clm = i-1;
                    next_Clm = i;
                end
                break;
            end
        end

        for i =1:length(Tb);
            if T < Tb(i);
                pre_T = Tb(i-1);
                next_T = Tb(i);
                pre_Row = i-1;
                next_Row = i;
                break;
            end
        end

        RotSpdScale = (curr_RotSpd - pre_RotSpd)/(next_RotSpd - pre_RotSpd);
        TScale = (T - pre_T)/(next_T - pre_T);
        if isnan(pre_Clm);
            pre_eff_dri = RotSpdScale * Eff(pre_Row, next_Clm);
            next_eff_dri = RotSpdScale * Eff(next_Row, next_Clm);
        else
            pre_eff_dri = RotSpdScale * ( Eff(pre_Row, next_Clm)-Eff(pre_Row, pre_Clm) ) + Eff(pre_Row, pre_Clm);
            next_eff_dri = RotSpdScale * ( Eff(next_Row, next_Clm)-Eff(next_Row, pre_Clm) ) + Eff(next_Row, pre_Clm);
        end

        eff_final = TScale * (next_eff_dri - pre_eff_dri) + pre_eff_dri;
          
    % driving
    else
        curr_RotSpd = 60*v/(2*pi*r);
        % get the two endpoints of the rotational speed interval
        for i =1:length(W_rpm);
            if curr_RotSpd < W_rpm(i);
                if i ==1;
                    pre_RotSpd = 0;
                    next_RotSpd = W_rpm(i);
                    pre_Clm = NaN;
                    next_Clm = i;
                else
                    pre_RotSpd = W_rpm(i-1);
                    next_RotSpd = W_rpm(i);
                    pre_Clm = i-1;
                    next_Clm = i;
                end
                break;
            end
        end

        for i =1:length(Tx);
            if T < Tx(i);
                pre_T = Tx(i-1);
                next_T = Tx(i);
                pre_Row = i-1;
                next_Row = i;
                break;
            end
        end

        RotSpdScale = (curr_RotSpd - pre_RotSpd)/(next_RotSpd - pre_RotSpd);
        TScale = (T - pre_T)/(next_T - pre_T);

        if isnan(pre_Clm);
            pre_eff_dri = RotSpdScale * eff_dri(pre_Row, next_Clm);
            next_eff_dri = RotSpdScale * eff_dri(next_Row, next_Clm);
        else
            pre_eff_dri = RotSpdScale * ( eff_dri(pre_Row, next_Clm)-eff_dri(pre_Row, pre_Clm) ) + eff_dri(pre_Row, pre_Clm);
            next_eff_dri = RotSpdScale * ( eff_dri(next_Row, next_Clm)-eff_dri(next_Row, pre_Clm) ) + eff_dri(next_Row, pre_Clm);
        end

        eff_final = TScale * (next_eff_dri - pre_eff_dri) + pre_eff_dri;
        
    end
%       check = eff_final
end

