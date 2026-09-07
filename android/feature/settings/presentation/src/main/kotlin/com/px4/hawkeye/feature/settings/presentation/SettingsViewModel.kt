package com.px4.hawkeye.feature.settings.presentation

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.px4.hawkeye.core.domain.PortValidationError
import com.px4.hawkeye.core.domain.onFailure
import com.px4.hawkeye.core.domain.onSuccess
import com.px4.hawkeye.core.domain.validateListenPort
import com.px4.hawkeye.core.presentation.UiText
import com.px4.hawkeye.feature.settings.domain.SettingsRepository
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch

class SettingsViewModel(private val repository: SettingsRepository) : ViewModel() {

    private val _state = MutableStateFlow(SettingsState())
    val state = _state.asStateFlow()

    init {
        viewModelScope.launch {
            repository.settings.collect { s ->
                _state.update {
                    it.copy(
                        themeMode = s.themeMode,
                        distanceUnit = s.distanceUnit,
                        // Only reseed the field from persistence while it holds a valid value,
                        // so an in-progress invalid edit isn't clobbered by the saved port.
                        portInput = if (it.portError == null) s.listenPort.toString() else it.portInput,
                        crashReportingEnabled = s.crashReportingEnabled,
                    )
                }
            }
        }
    }

    fun onAction(action: SettingsAction) {
        when (action) {
            is SettingsAction.OnThemeModeSelected ->
                viewModelScope.launch { repository.setThemeMode(action.mode) }
            is SettingsAction.OnDistanceUnitSelected ->
                viewModelScope.launch { repository.setDistanceUnit(action.unit) }
            is SettingsAction.OnListenPortChanged -> onListenPortChanged(action.raw)
            is SettingsAction.OnCrashReportingToggled ->
                viewModelScope.launch { repository.setCrashReportingEnabled(action.enabled) }
        }
    }

    private fun onListenPortChanged(raw: String) {
        _state.update { it.copy(portInput = raw) }
        validateListenPort(raw)
            .onSuccess { port ->
                _state.update { it.copy(portError = null) }
                viewModelScope.launch { repository.setListenPort(port) }
            }
            .onFailure { error ->
                _state.update { it.copy(portError = error.toUiText()) }
            }
    }
}

private fun PortValidationError.toUiText(): UiText = when (this) {
    PortValidationError.NOT_A_NUMBER -> UiText.StringResource(R.string.settings_port_error_number)
    PortValidationError.OUT_OF_RANGE -> UiText.StringResource(R.string.settings_port_error_range)
}
