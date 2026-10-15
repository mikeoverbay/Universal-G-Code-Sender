package com.willwinder.universalgcodesender.firmware.fluidnc;

import com.willwinder.universalgcodesender.communicator.ICommunicator;
import com.willwinder.universalgcodesender.firmware.fluidnc.commands.FluidNCCommand;
import com.willwinder.universalgcodesender.gcode.GcodeState;
import com.willwinder.universalgcodesender.gcode.util.Code;
import com.willwinder.universalgcodesender.listeners.ControllerListener;
import com.willwinder.universalgcodesender.listeners.ControllerState;
import com.willwinder.universalgcodesender.listeners.ControllerStatus;
import com.willwinder.universalgcodesender.listeners.ControllerStatusBuilder;
import com.willwinder.universalgcodesender.listeners.MessageType;
import com.willwinder.universalgcodesender.model.PartialPosition;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UnitUtils;
import com.willwinder.universalgcodesender.services.MessageService;
import com.willwinder.universalgcodesender.types.GcodeCommand;
import com.willwinder.universalgcodesender.utils.IGcodeStreamReader;
import com.willwinder.universalgcodesender.utils.SimpleGcodeStreamReader;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.ArgumentMatchers.startsWith;
import org.mockito.InOrder;
import org.mockito.Mock;
import static org.mockito.Mockito.doNothing;
import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoMoreInteractions;
import static org.mockito.Mockito.when;
import org.mockito.MockitoAnnotations;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.List;

public class FluidNCControllerTest {
    private static final String WELCOME_STRING = "Grbl 3.7 [FluidNC v3.7.8 (noradio) '$' for help]";
    private static final String IDLE_STATUS = "<Idle|MPos:0.000,0.000,0.000|FS:0,0>";

    private FluidNCController target;
    private ICommunicator communicator;

    @Mock
    private MessageService messageService;

    @Before
    public void setUp() {
        MockitoAnnotations.openMocks(this);
        communicator = mock(ICommunicator.class);
        target = spy(new FluidNCController(communicator));
        target.setMessageService(messageService);
    }

    @Test
    public void executeReturnToHomeShouldAddSafetyHeightWhenBelow() throws Exception {
        when(target.isIdle()).thenReturn(true);
        mockGcodeState();
        mockControllerStatus(ControllerState.IDLE, new Position(0, 0, 5, UnitUtils.Units.MM));

        ArgumentCaptor<GcodeCommand> commandArgumentCaptor = ArgumentCaptor.forClass(GcodeCommand.class);
        doNothing().when(target).sendCommandImmediately(commandArgumentCaptor.capture());

        target.returnToHome(10);

        verify(target, times(3)).sendCommandImmediately(any());
        List<GcodeCommand> commands = commandArgumentCaptor.getAllValues();
        assertEquals("G21G90 G0Z10", commands.get(0).getCommandString());
        assertEquals("G90 G0 X0 Y0", commands.get(1).getCommandString());
        assertEquals("G90 G0 Z0", commands.get(2).getCommandString());
    }

    @Test
    public void executeReturnToHomeShouldNotAddSafetyHeightWhenOver() throws Exception {
        when(target.isIdle()).thenReturn(true);
        mockGcodeState();
        mockControllerStatus(ControllerState.IDLE, new Position(0, 0, 11, UnitUtils.Units.MM));

        ArgumentCaptor<GcodeCommand> commandArgumentCaptor = ArgumentCaptor.forClass(GcodeCommand.class);
        doNothing().when(target).sendCommandImmediately(commandArgumentCaptor.capture());

        target.returnToHome(10);

        verify(target, times(2)).sendCommandImmediately(any());
        List<GcodeCommand> commands = commandArgumentCaptor.getAllValues();
        assertEquals("G90 G0 X0 Y0", commands.get(0).getCommandString());
        assertEquals("G90 G0 Z0", commands.get(1).getCommandString());
    }

    @Test
    public void restoreParserModalStateShouldRestoreUnitsToMM() {
        target.updateParserModalState(new GcodeCommand("G21"));

        ArgumentCaptor<GcodeCommand> commandArgumentCaptor = ArgumentCaptor.forClass(GcodeCommand.class);
        doNothing().when(communicator).queueCommand(commandArgumentCaptor.capture());
        target.restoreParserModalState();

        assertEquals("G21", commandArgumentCaptor.getValue().getCommandString());
    }

    @Test
    public void restoreParserModalStateShouldRestoreUnitsToInches() {
        target.updateParserModalState(new GcodeCommand("G20"));

        ArgumentCaptor<GcodeCommand> commandArgumentCaptor = ArgumentCaptor.forClass(GcodeCommand.class);
        doNothing().when(communicator).queueCommand(commandArgumentCaptor.capture());
        target.restoreParserModalState();

        assertEquals("G20", commandArgumentCaptor.getValue().getCommandString());
    }

    @Test
    public void restoreParserModalStateShouldRestoreAbsoluteMode() {
        target.updateParserModalState(new GcodeCommand("G90"));

        ArgumentCaptor<GcodeCommand> commandArgumentCaptor = ArgumentCaptor.forClass(GcodeCommand.class);
        doNothing().when(communicator).queueCommand(commandArgumentCaptor.capture());
        target.restoreParserModalState();

        assertEquals("G90", commandArgumentCaptor.getValue().getCommandString());
    }

    @Test
    public void beginStreamingShouldSendEvents() {
        ControllerListener listener = mock(ControllerListener.class);
        target.addListener(listener);

        IGcodeStreamReader gcodeStream = new SimpleGcodeStreamReader("G0 X1", "G0 X0");
        target.queueStream(gcodeStream);
        target.beginStreaming();

        assertTrue(target.isStreaming());
        verify(listener, times(1)).streamStarted();
        verify(listener, times(1)).statusStringListener(any());
        verifyNoMoreInteractions(listener);
    }

    @Test
    public void streamCompleteShouldBeExecutedWhenStreamIsFinished() throws IOException, InterruptedException {
        ControllerListener listener = mock(ControllerListener.class);
        InOrder inOrder = inOrder(listener);
        target.addListener(listener);

        IGcodeStreamReader gcodeStream = new SimpleGcodeStreamReader("G0 X1", "G0 X0");
        target.queueStream(gcodeStream);
        target.beginStreaming();

        GcodeCommand nextCommand = gcodeStream.getNextCommand();
        target.commandSent(nextCommand);
        nextCommand.appendResponse("ok");
        target.rawResponseListener("ok");

        nextCommand = gcodeStream.getNextCommand();
        target.commandSent(nextCommand);
        nextCommand.appendResponse("ok");
        target.rawResponseListener("ok");

        Thread.sleep(100);

        inOrder.verify(listener, times(1)).statusStringListener(any());
        inOrder.verify(listener, times(1)).streamStarted();
        inOrder.verify(listener, times(1)).commandSent(any());
        inOrder.verify(listener, times(1)).commandComplete(any());
        inOrder.verify(listener, times(1)).commandSent(any());
        inOrder.verify(listener, times(1)).commandComplete(any());
        inOrder.verify(listener, times(1)).streamComplete();
        inOrder.verifyNoMoreInteractions();
    }

    @Test
    public void rawResponseListenerShouldReportErrorOnCommandIfThereAreMoreActiveCommands() throws IOException, InterruptedException {
        ControllerListener listener = mock(ControllerListener.class);
        InOrder messagesInOrder = inOrder(messageService);
        InOrder inOrder = inOrder(listener);
        target.addListener(listener);

        IGcodeStreamReader gcodeStream = new SimpleGcodeStreamReader("G0 X1", "G0 X0");
        target.queueStream(gcodeStream);
        target.beginStreaming();

        GcodeCommand nextCommand = gcodeStream.getNextCommand();
        nextCommand.appendResponse("error:20");
        target.commandSent(nextCommand);

        nextCommand = gcodeStream.getNextCommand();
        target.commandSent(nextCommand);

        target.rawResponseListener("error:20");

        Thread.sleep(100);

        inOrder.verify(listener, times(1)).statusStringListener(any());
        inOrder.verify(listener, times(1)).streamStarted();
        inOrder.verify(listener, times(2)).commandSent(any());
        inOrder.verify(listener, times(1)).commandComplete(any());
        inOrder.verify(listener, times(1)).streamComplete();
        inOrder.verifyNoMoreInteractions();

        messagesInOrder.verify(messageService, times(1)).dispatchMessage(MessageType.INFO, "> G0 X1\n");
        messagesInOrder.verify(messageService, times(1)).dispatchMessage(MessageType.ERROR, "An error was detected while sending 'G0 X1': error:20. Streaming has been paused.\n");
        messagesInOrder.verify(messageService, times(1)).dispatchMessage(eq(MessageType.INFO), startsWith("\n**** Finished sending file in"));
        messagesInOrder.verifyNoMoreInteractions();
    }

    @Test
    public void rawResponseListenerShouldNotReportErrorOnCommandIfOnlyOneActiveCommands() throws IOException, InterruptedException {
        ControllerListener listener = mock(ControllerListener.class);
        InOrder messagesInOrder = inOrder(messageService);
        InOrder inOrder = inOrder(listener);
        target.addListener(listener);

        IGcodeStreamReader gcodeStream = new SimpleGcodeStreamReader("G0 X1");
        target.queueStream(gcodeStream);
        target.beginStreaming();

        GcodeCommand nextCommand = gcodeStream.getNextCommand();
        nextCommand.appendResponse("error:20");
        target.commandSent(nextCommand);

        target.rawResponseListener("error:20");

        Thread.sleep(100);

        inOrder.verify(listener, times(1)).statusStringListener(any());
        inOrder.verify(listener, times(1)).streamStarted();
        inOrder.verify(listener, times(1)).commandSent(any());
        inOrder.verify(listener, times(1)).commandComplete(any());
        inOrder.verify(listener, times(1)).streamComplete();
        inOrder.verifyNoMoreInteractions();

        messagesInOrder.verify(messageService, times(1)).dispatchMessage(MessageType.INFO, "> G0 X1\n");
        messagesInOrder.verify(messageService, times(1)).dispatchMessage(MessageType.INFO, "error:20\n");
        messagesInOrder.verify(messageService, times(1)).dispatchMessage(eq(MessageType.INFO), startsWith("\n**** Finished sending file in"));
        messagesInOrder.verifyNoMoreInteractions();
    }

    @Test
    public void restoreParserModalStateShouldRestoreRelativeMode() {
        target.updateParserModalState(new GcodeCommand("G91"));

        ArgumentCaptor<GcodeCommand> commandArgumentCaptor = ArgumentCaptor.forClass(GcodeCommand.class);
        doNothing().when(communicator).queueCommand(commandArgumentCaptor.capture());
        target.restoreParserModalState();

        assertEquals("G91", commandArgumentCaptor.getValue().getCommandString());
    }

    @Test
    public void jogCommandsShouldNotChangeTheRestoredGcodeState() throws Exception {
        target.updateParserModalState(new GcodeCommand("G90 G21"));

        ArgumentCaptor<GcodeCommand> commandArgumentCaptor = ArgumentCaptor.forClass(GcodeCommand.class);
        doNothing().when(communicator).queueCommand(commandArgumentCaptor.capture());
        target.jogMachine(new PartialPosition(1.0, null, null, UnitUtils.Units.INCH), 100);
        target.jogMachineTo(new PartialPosition(0.0, 0.0, UnitUtils.Units.INCH), 100);

        // The units and distance mode in a jog only apply to the jog
        List<GcodeCommand> jogCommands = commandArgumentCaptor.getAllValues();
        assertEquals("$J=G20G91X1F100", jogCommands.get(0).getCommandString());
        jogCommands.forEach(command -> {
            assertTrue(command.isTemporaryParserModalChange());
            target.updateParserModalState(command);
        });

        target.restoreParserModalState();
        assertEquals("G90 G21", commandArgumentCaptor.getValue().getCommandString());
    }

    @Test
    public void onConnectionClosedShouldDisconnectController() throws Exception {
        when(target.isIdle()).thenReturn(true);
        target.rawResponseListener("<Idle>");

        assertEquals(ControllerState.IDLE, target.getControllerStatus().getState());

        target.onConnectionClosed();

        verify(communicator, times(1)).disconnect();
        assertEquals(ControllerState.DISCONNECTED, target.getControllerStatus().getState());
    }

    @Test
    public void welcomeStringWhenInitializedShouldSetInchUnitsOnceWhenIdle() throws Exception {
        givenAnInitializedController();

        target.rawResponseListener(WELCOME_STRING);
        verify(communicator, never()).queueCommand(any());

        target.rawResponseListener(IDLE_STATUS);
        target.rawResponseListener(IDLE_STATUS);

        GcodeCommand command = captureQueuedCommand();
        verify(communicator, times(1)).streamCommands();
        assertEquals("G20", command.getCommandString());
        assertFalse(command.isTemporaryParserModalChange());
    }

    @Test
    public void inchUnitsShouldNotBeSetUntilTheControllerIsIdle() throws Exception {
        givenAnInitializedController();
        target.rawResponseListener(WELCOME_STRING);

        target.rawResponseListener("<Alarm|MPos:0.000,0.000,0.000|FS:0,0>");
        target.rawResponseListener("<Home|MPos:0.000,0.000,0.000|FS:0,0>");
        target.rawResponseListener("<Jog|MPos:0.000,0.000,0.000|FS:0,0>");
        target.rawResponseListener("<Run|MPos:0.000,0.000,0.000|FS:0,0>");
        target.rawResponseListener("<Hold:0|MPos:0.000,0.000,0.000|FS:0,0>");
        target.rawResponseListener("<Door:0|MPos:0.000,0.000,0.000|FS:0,0>");
        target.rawResponseListener("<Check|MPos:0.000,0.000,0.000|FS:0,0>");
        target.rawResponseListener("<Sleep|MPos:0.000,0.000,0.000|FS:0,0>");
        verify(communicator, never()).queueCommand(any());

        target.rawResponseListener(IDLE_STATUS);
        verify(communicator, times(1)).queueCommand(any());
    }

    @Test
    public void inchUnitsShouldNotBeSetWhileCommandsAreActive() throws Exception {
        givenAnInitializedController();
        target.rawResponseListener(WELCOME_STRING);

        when(communicator.areActiveCommands()).thenReturn(true);
        target.rawResponseListener(IDLE_STATUS);
        when(communicator.areActiveCommands()).thenReturn(false);

        when(communicator.isPaused()).thenReturn(true);
        target.rawResponseListener(IDLE_STATUS);
        when(communicator.isPaused()).thenReturn(false);

        // A command still waiting for its response, like a homing cycle
        GcodeCommand homingCommand = new FluidNCCommand("$H");
        target.commandSent(homingCommand);
        target.rawResponseListener(IDLE_STATUS);
        verify(communicator, never()).queueCommand(any());

        homingCommand.appendResponse("ok");
        target.rawResponseListener("ok");
        target.rawResponseListener(IDLE_STATUS);
        verify(communicator, times(1)).queueCommand(any());
    }

    @Test
    public void inchUnitsShouldUpdateTheGcodeStateWhenOk() throws Exception {
        givenAnInitializedController();
        target.rawResponseListener(WELCOME_STRING);
        target.rawResponseListener(IDLE_STATUS);
        GcodeCommand command = captureQueuedCommand();
        assertEquals(UnitUtils.Units.MM, target.getCurrentGcodeState().getUnits());

        target.commandSent(command);
        command.appendResponse("ok");
        target.rawResponseListener("ok");
        assertEquals(UnitUtils.Units.INCH, target.getCurrentGcodeState().getUnits());

        // Restoring the modal state after for instance probing should keep inches
        target.restoreParserModalState();
        ArgumentCaptor<GcodeCommand> commandArgumentCaptor = ArgumentCaptor.forClass(GcodeCommand.class);
        verify(communicator, times(2)).queueCommand(commandArgumentCaptor.capture());
        assertEquals("G90 G20", commandArgumentCaptor.getValue().getCommandString());
    }

    @Test
    public void welcomeStringWhenInitializedShouldResetTheGcodeStateToMillimeters() throws Exception {
        givenAnInitializedController();
        target.updateParserModalState(new GcodeCommand("G20 G91"));

        target.rawResponseListener(WELCOME_STRING);
        assertEquals(UnitUtils.Units.MM, target.getCurrentGcodeState().getUnits());
        assertEquals(Code.G90, target.getCurrentGcodeState().distanceMode);

        // Restoring the modal state after for instance probing should restore what the device has after a reset
        target.restoreParserModalState();
        assertEquals("G90 G21", captureQueuedCommand().getCommandString());
    }

    @Test
    public void welcomeStringWhileACommandIsActiveShouldSetInchUnitsWhenTheCommandIsCleared() throws Exception {
        givenAnInitializedController();
        target.updateParserModalState(new GcodeCommand("G20"));

        // A reset while for instance probing, the welcome string is added to the command that was in flight
        GcodeCommand probeCommand = new FluidNCCommand("G38.2 Z-1 F10");
        target.commandSent(probeCommand);
        probeCommand.appendResponse(WELCOME_STRING);
        target.rawResponseListener(WELCOME_STRING);
        assertEquals(UnitUtils.Units.MM, target.getCurrentGcodeState().getUnits());

        target.rawResponseListener(IDLE_STATUS);
        verify(communicator, never()).queueCommand(any());

        // Cleared when the user stops
        target.resetBuffers();
        target.rawResponseListener(IDLE_STATUS);
        assertEquals("G20", captureQueuedCommand().getCommandString());
    }

    @Test
    public void messagesMatchingTheWelcomePatternShouldNotResetTheGcodeState() throws Exception {
        givenAnInitializedController();
        target.updateParserModalState(new GcodeCommand("G20"));

        target.rawResponseListener("[MSG:Use bit [end mill] now]");
        target.rawResponseListener(IDLE_STATUS);

        assertEquals(UnitUtils.Units.INCH, target.getCurrentGcodeState().getUnits());
        verify(communicator, never()).queueCommand(any());
    }

    @Test
    public void inchUnitsShouldBeRetriedAFewTimesWithoutChangingTheGcodeStateOnError() throws Exception {
        givenAnInitializedController();
        target.updateParserModalState(new GcodeCommand("G20"));
        target.rawResponseListener(WELCOME_STRING);

        ArgumentCaptor<GcodeCommand> commandArgumentCaptor = ArgumentCaptor.forClass(GcodeCommand.class);
        for (int attempt = 1; attempt <= 3; attempt++) {
            target.rawResponseListener(IDLE_STATUS);
            verify(communicator, times(attempt)).queueCommand(commandArgumentCaptor.capture());
            GcodeCommand command = commandArgumentCaptor.getValue();
            assertEquals("G20", command.getCommandString());

            // Rejected, for instance if a jog was started at the same time
            target.commandSent(command);
            command.appendResponse("error:9");
            target.rawResponseListener("error:9");
            assertEquals(UnitUtils.Units.MM, target.getCurrentGcodeState().getUnits());
        }

        target.rawResponseListener(IDLE_STATUS);
        verify(communicator, times(3)).queueCommand(any());
    }

    @Test
    public void inchUnitsShouldNotBeSetWithoutAConnectionOrReset() {
        when(communicator.isConnected()).thenReturn(true);

        target.rawResponseListener(IDLE_STATUS);

        verify(communicator, never()).queueCommand(any());
    }

    @Test
    public void closeCommPortShouldCancelPendingInchUnits() throws Exception {
        givenAnInitializedController();
        target.rawResponseListener(WELCOME_STRING);

        target.closeCommPort();
        target.rawResponseListener(IDLE_STATUS);

        verify(communicator, never()).queueCommand(any());
    }

    private void givenAnInitializedController() throws Exception {
        when(communicator.isConnected()).thenReturn(true);

        // Initializing needs a real device, set the flag instead. A welcome string while not initialized would start it.
        Field isInitialized = FluidNCController.class.getDeclaredField("isInitialized");
        isInitialized.setAccessible(true);
        isInitialized.set(target, true);
    }

    private GcodeCommand captureQueuedCommand() {
        ArgumentCaptor<GcodeCommand> commandArgumentCaptor = ArgumentCaptor.forClass(GcodeCommand.class);
        verify(communicator, times(1)).queueCommand(commandArgumentCaptor.capture());
        return commandArgumentCaptor.getValue();
    }

    private void mockControllerStatus(ControllerState state, Position workPosition) {
        ControllerStatus controllerStatus = ControllerStatusBuilder.newInstance()
                .setState(state)
                .setWorkCoord(workPosition)
                .build();
        when(target.getControllerStatus()).thenReturn(controllerStatus);
    }

    private void mockGcodeState() {
        GcodeState state = new GcodeState();
        state.units = Code.G21;
        when(target.getCurrentGcodeState()).thenReturn(state);
    }
}
