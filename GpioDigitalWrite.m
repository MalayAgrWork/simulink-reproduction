classdef GpioDigitalWrite < matlab.System ...
        & coder.ExternalDependency ...
        & matlabshared.sensors.simulink.internal.BlockSampleTime

    %
    %#codegen
    %#ok<*EMCA>

    properties

    end

    properties(Access = protected)
        Logo = 'App generated';
    end

    properties (Nontunable)

    end

    properties (Access = private)


    end

    methods
        % Constructor
        function obj = GpioDigitalWrite(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end

    methods (Access=protected)
        function setupImpl(obj)
            if ~coder.target('MATLAB')
                coder.cinclude('GpioDigitalWrite.h');
                coder.ceval('setupFunctionGpioDigitalWrite');
            end
        end

        function stepImpl(obj ,Input)

            if isempty(coder.target)
            else
                coder.ceval('stepFunctionGpioDigitalWrite', Input,1);
            end
        end

        function releaseImpl(obj)
            if isempty(coder.target)
            else

            end
        end
    end

    methods (Access=protected)
        %% Define output properties
        function num = getNumInputsImpl(~)
            num = 1;
        end

        function num = getNumOutputsImpl(~)
            num = 0;
        end

        function varargout = getInputNamesImpl(obj)
            varargout{1} = 'Input';

        end

        function varargout = getOutputNamesImpl(obj)

        end

        function flag = isOutputSizeLockedImpl(~,~)
            flag = true;
        end

        function varargout = isOutputFixedSizeImpl(~,~)

        end

        function varargout = isOutputComplexImpl(~)

        end

        function varargout = getOutputSizeImpl(~)

        end

        function varargout = getOutputDataTypeImpl(~)

        end

        function maskDisplayCmds = getMaskDisplayImpl(obj)
            outport_label = [];
            num = getNumOutputsImpl(obj);
            if num > 0
                outputs = cell(1,num);
                [outputs{1:num}] = getOutputNamesImpl(obj);
                for i = 1:num
                    outport_label = [outport_label 'port_label(''output'',' num2str(i) ',''' outputs{i} ''');' ]; %#ok<AGROW>
                end
            end
            inport_label = [];
            num = getNumInputsImpl(obj);
            if num > 0
                inputs = cell(1,num);
                [inputs{1:num}] = getInputNamesImpl(obj);
                for i = 1:num
                    inport_label = [inport_label 'port_label(''input'',' num2str(i) ',''' inputs{i} ''');' ]; %#ok<AGROW>
                end
            end
            icon = 'GpioDigitalWrite';
            maskDisplayCmds = [ ...
                ['color(''white'');',...
                'plot([100,100,100,100]*1,[100,100,100,100]*1);',...
                'plot([100,100,100,100]*0,[100,100,100,100]*0);',...
                'color(''blue'');', ...
                ['text(38, 92, ','''',obj.Logo,'''',',''horizontalAlignment'', ''right'');',newline],...
                'color(''black'');'], ...
                ['text(52,50,' [''' ' icon ''',''horizontalAlignment'',''center'');' newline]]   ...
                inport_label ...
                outport_label
                ];
        end

        function sts = getSampleTimeImpl(obj)
            sts = getSampleTimeImpl@matlabshared.sensors.simulink.internal.BlockSampleTime(obj);
        end
    end

    methods (Static, Access=protected)
        function simMode = getSimulateUsingImpl(~)
            simMode = 'Interpreted execution';
        end

        function isVisible = showSimulateUsingImpl
            isVisible = false;
        end
    end

    methods (Static)
        function name = getDescriptiveName()
            name = 'GpioDigitalWrite';
        end

        function b = isSupportedContext(context)
            b = context.isCodeGenTarget('rtw');
        end

        function updateBuildInfo(buildInfo, context)
            % coder.extrinsic('matlabshared.sensors.simulink.internal.getTargetHardwareName');
            % targetname = coder.const(matlabshared.sensors.simulink.internal.getTargetHardwareName);
            % % Get the filelocation of the SPKG specific files
            % coder.extrinsic('matlabshared.sensors.simulink.internal.getTargetSpecificFileLocationForSensors');
            % fileLocation = coder.const(@matlabshared.sensors.simulink.internal.getTargetSpecificFileLocationForSensors,targetname);
            % coder.extrinsic('which');
            % coder.extrinsic('error');
            % coder.extrinsic('message');
            % funcName = [fileLocation,'.getTargetSensorUtilities'];
            % functionPath = coder.const(@which,funcName);
            % % Only if the the path exist
            % if ~isempty(fileLocation)
            %     % internal error to see if the target author has provided
            %     % the expected function in the specified file location
            %     assert(~isempty(functionPath),message('matlab_sensors:general:FunctionNotAvailableSimulinkSensors','getTargetSensorUtilities'));
            %     funcHandle = str2func(funcName);
            %     hwUtilityObject = funcHandle('I2C');
            %     assert(isa(hwUtilityObject,'matlabshared.sensors.simulink.internal.SensorSimulinkBase'),message('matlab_sensors:general:invalidHwObjSensorSimulink'));
            % else
            %     hwUtilityObject = '';
            % end
            % hwUtilityObject.updateBuildInfo(buildInfo, context);
            buildInfo.addIncludePaths('D:\milestone2DeviceBuilder\Drivers\STM32F4xx_HAL_Driver\Src');
            % 
            % addSourceFiles(buildInfo, D:\milestone2DeviceBuilder\Drivers\STM32F4xx_HAL_Driver\Src');
            addSourceFiles(buildInfo,'GpioDigitalWrite.cpp','D:\milestone2DeviceBuilder');

        end
    end
end
