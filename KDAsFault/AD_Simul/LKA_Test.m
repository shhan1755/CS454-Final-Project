t = tcpip('127.0.0.1', 10004, 'NetworkRole', 'server');
fopen(t);

while 1
    if (get(t,'BytesAvailable') > 0)
        bytes = fread(t, [1, t.BytesAvailable]);
        if (char(bytes) == 'q')
            break;
        else
           %% Preprocess
            recv_name = (char(bytes));
            filename = ['CSV_data/', recv_name, '.csv'];
            disp(filename)
            data1 = csvread(filename);
            %data_table = readtable(filename);
            %data1 = table2array(data_table);
            speed = 9.8; %ego speed (m/s), 13.9 m/s = 50 kph
            roadWidth = 4;
            try 
                [scenario, waypoints, egoVehicle] = createDrivingScenario(speed, data1, roadWidth);
            catch
                disp('error1')
                max_deviation = -1;
                fwrite(t, num2str(max_deviation));
                close all
                bdclose all
                continue;
            end
                %drivingScenarioDesigner(scenario)

            %%
            open_system('LKATestBenchExample')

            set_param('LKATestBenchExample/Enable','Value','1') 
            set_param('LKATestBenchExample/Safe Lateral Offset','Value','0.5')
            %%
            plotLKAInputs(scenario,driverPath)
            errorflag = 0;

            %% 
            try 
                sim('LKATestBenchExample')                  % Simulate to end of scenario
            catch
                disp('error')
                errorflag = 1;
            end

            %%
            plotLKAResults(scenario,logsout,driverPath, recv_name)

            %%
            % To plot the controller performance, use the following command.
            if (errorflag == 0)
                max_deviation = plotLKAPerformance(logsout);
                stream = ['max deviation: ', num2str(max_deviation)];
                disp(stream)
                plotLKAStatus(logsout)
                disp(max_deviation);
                fwrite(t, num2str(max_deviation));
            elseif (errorflag == 1)
                max_deviation = -1;
                stream = ['max deviation: ', num2str(max_deviation)];
                disp(stream)
                plotLKAStatus(logsout)
                disp(max_deviation);
                fwrite(t, num2str(max_deviation));
            end

           %%
            close all
            bdclose all
        end
    end
end

fclose(t);
        