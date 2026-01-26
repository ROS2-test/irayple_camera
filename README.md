Скрипт для парсинга qr-кодов с пола

Предварительно установим камеру как сервер

<details>
<summary>
Параметры камеры (устанавливаются в виндовсном приложении).
Можно создать файл со следующим наполнением и импортировать в приложении с камерой.
</summary>

    <?xml version="1.0" encoding="UTF-8" standalone="yes" ?>
    <Root>
        <Manifest>
            <DeviceModel>R3138MG010E</DeviceModel>
        </Manifest>
        <CameraConfig>
            <DeviceControl type="group" value="">
                <DeviceVendorName type="string" value="Huaray Technology" />
                <DeviceModelName type="string" value="R3138MG010E" />
                <DeviceManufacturerInfo type="string" value="Huaray Technology" />
                <DeviceVersion type="string" value="V1.003.0000.F.R.20240826-1144" />
                <DeviceFirmwareVersion type="string" value="SYSTEM(274177)" />
                <DeviceSoftwareVersion type="string" value="FPGA(4.4.2)" />
                <AlgorithmVersion type="string" value="BarCode_3.2.4.249126&#x09;Datacode_2.9.323193  NNIE_2.10.A.0" />
                <DeviceSerialNumber type="string" value="EG21932AAK00020" />
                <DeviceUserID type="string" value="" />
            </DeviceControl>
            <TransportLayerControl type="group" value="">
                <ScpdEnable type="bool" value="0" />
            </TransportLayerControl>
            <NTPControl type="group" value="">
                <NTPServerIPAddress type="int" value="0" />
                <NTPEnableCyclicSyn type="bool" value="0" />
                <NTPSynTriggerOnce type="command" value="" />
            </NTPControl>
            <ImageFormatControl type="group" value="">
                <SensorWidth type="int" value="544" />
                <SensorHeight type="int" value="640" />
                <WidthMax type="int" value="544" />
                <HeightMax type="int" value="640" />
                <PixelFormat type="enum" value="17301505" />
                <TestImageSelector type="enum" value="0" />
            </ImageFormatControl>
            <TriggerControl type="group" value="">
                <TriggerType type="enum" value="0" />
                <FrameRate type="float" value="120.000000" />
                <FrameRateMax type="float" value="120.000000" />
                <TriggerSource type="enum" value="0" />
            </TriggerControl>
            <ExposureControl type="group" value="">
                <ExposureMode type="enum" value="0" />
                <ExposureTime type="float" value="350.000000" />
                <GainRaw type="float" value="12.000000" />
            </ExposureControl>
            <LightControl type="group" value="">
                <Selector name="LightSelector" value="0">
                    <LightBrightness type="int" value="80" />
                    <LightMode type="enum" value="2" />
                    <LightPrechargeTime type="float" value="150.000000" />
                </Selector>
            </LightControl>
            <DigitalIOControl type="group" value="">
                <Selector name="LineSelector" value="1">
                    <LineMode type="enum" value="1" />
                    <LineOutput type="enum" value="0" />
                    <LineOutputInvertEnable type="bool" value="0" />
                    <LineOutputPulseWidth type="float" value="30.000000" />
                    <LineOutputSource type="enum" value="0" />
                    <LineOutputTriggerDelay type="float" value="0.000000" />
                    <LineOutputType type="enum" value="0" />
                    <LineStatus type="enum" value="0" />
                </Selector>
                <Selector name="LineSelector" value="0">
                    <LineInputDebounceEnable type="bool" value="1" />
                    <LineInputDebounceFilter type="float" value="1000.000000" />
                    <LineInputInvertEnable type="bool" value="0" />
                    <LineMode type="enum" value="0" />
                    <LineStatus type="enum" value="0" />
                </Selector>
            </DigitalIOControl>
            <ImageSourceControl type="group" value="">
                <ImageSource type="enum" value="0" />
                <ImageUrl type="string" value="" />
                <ImageName type="string" value="" />
            </ImageSourceControl>
            <ImageInputControl type="group" value="">
                <InputProcEnable type="bool" value="0" />
                <VerticalFlip type="bool" value="0" />
                <HorizontalFilp type="bool" value="0" />
            </ImageInputControl>
            <ImageOutputControl type="group" value="">
                <ScaleEnable type="bool" value="0" />
            </ImageOutputControl>
            <VisionControl type="group" value="">
                <WorkMode type="enum" value="1" />
                <VisionFuntion type="enum" value="0" />
                <FrmInBuffNr type="int" value="2" />
            </VisionControl>
            <MatrixCodeProcessControl type="group" value="">
                <EnableMatrixCodeConfig type="enum" value="1" />
                <DMEnable type="bool" value="1" />
                <ResultFilterMatrixCode type="bool" value="1" />
                <MinMatrixCodeLen type="int" value="0" />
                <MaxMatrixCodeLen type="int" value="0" />
                <MaxOffsetLShapeAngle type="float" value="2.000000" />
                <MaxOffsetOtherAngle type="float" value="2.000000" />
                <PatternMatrixCode type="string" value="" />
                <IsCodeMatrix type="bool" value="1" />
                <EnableBoxCorrection type="bool" value="1" />
                <SignPolarity type="enum" value="2" />
                <SignMirror type="enum" value="0" />
                <VersionMinDM type="int" value="1" />
                <VersionMaxDM type="int" value="3" />
                <CodeMinWidth type="int" value="60" />
                <CodeMaxWidth type="int" value="400" />
                <DecodeNum type="int" value="1" />
                <TimeOutMatrixCode type="int" value="20" />
                <DecoderCapacity type="enum" value="0" />
            </MatrixCodeProcessControl>
            <AGVControl type="group" value="">
                <AGVEnable type="bool" value="1" />
                <SiteID type="int" value="0" />
                <CommMode type="enum" value="1" />
                <ResultPackMode type="enum" value="0" />
                <ResultTransferStyle type="enum" value="1" />
                <StopSendNoRead type="bool" value="0" />
                <Timeout type="int" value="1000" />
                <DmLenMM type="float" value="15.000000" />
                <DmIntervalLenMM type="float" value="5.000000" />
                <ResolutionX type="enum" value="1" />
                <ResolutionY type="enum" value="1" />
                <ResolutionAngle type="enum" value="1" />
                <OffsetAngle type="int" value="0" />
                <TagSize type="enum" value="1" />
            </AGVControl>
            <SerialControl type="group" value="">
                <BaudRate type="enum" value="115200" />
                <DataBits type="enum" value="3" />
                <Parity type="enum" value="0" />
                <StopBits type="enum" value="0" />
            </SerialControl>
            <EthernetTransferCtl type="group" value="">
                <TransferEnable type="bool" value="1" />
                <TransferWorkMode type="enum" value="0" />
                <TCPPort type="int" value="3000" />
            </EthernetTransferCtl>
            <UserSetControl type="group" value="">
                <UserSetSelector type="enum" value="1" />
                <UserSetLoad type="command" value="" />
                <UserSetSave type="command" value="" />
                <UserSetDefault type="enum" value="0" />
                <UserSetLoadLastUserSet type="enum" value="0" />
                <UserSetLoadStatus type="enum" value="0" />
            </UserSetControl>
            <Debug type="group" value="">
                <TriggerLastFrame type="command" value="" />
                <DebugImageExport type="enum" value="0" />
                <ClickExportImage type="command" value="" />
            </Debug>
        </CameraConfig>
    </Root>

</details>

Скрипт ловит данные с камеры по TCP,
декодирует в кортеж из 4х элементов:
(int, int, int, str)

и отправляет в топик irayple
работает на ros2 jazzy
тип сообщений хранится в irayple_mgs

Запуск:
```sh
cd /home/user/ros2_ws
source install/setup.bash
ros2 run irayple irayple_parser
```