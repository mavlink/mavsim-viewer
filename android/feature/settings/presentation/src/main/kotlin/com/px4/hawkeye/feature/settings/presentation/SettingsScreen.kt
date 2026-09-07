package com.px4.hawkeye.feature.settings.presentation

import androidx.annotation.StringRes
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.statusBarsPadding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.selection.selectable
import androidx.compose.foundation.selection.toggleable
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.RadioButton
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.semantics.Role
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.tooling.preview.Preview
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.px4.hawkeye.core.designsystem.HawkeyeDimens
import com.px4.hawkeye.core.designsystem.HawkeyeTheme
import com.px4.hawkeye.core.presentation.asString
import com.px4.hawkeye.feature.settings.domain.DistanceUnit
import com.px4.hawkeye.feature.settings.domain.ThemeMode
import org.koin.androidx.compose.koinViewModel

@Composable
fun SettingsRoot(
    onNavigateToAbout: () -> Unit,
    viewModel: SettingsViewModel = koinViewModel(),
) {
    val state by viewModel.state.collectAsStateWithLifecycle()
    SettingsScreen(
        state = state,
        onAction = viewModel::onAction,
        onNavigateToAbout = onNavigateToAbout,
    )
}

@Composable
fun SettingsScreen(
    state: SettingsState,
    onAction: (SettingsAction) -> Unit,
    onNavigateToAbout: () -> Unit,
) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .statusBarsPadding()
            .verticalScroll(rememberScrollState())
            .padding(HawkeyeDimens.contentPadding),
    ) {
        Text(
            text = stringResource(R.string.settings_theme_header),
            style = MaterialTheme.typography.titleMedium,
            modifier = Modifier.padding(bottom = HawkeyeDimens.titleSpacing),
        )
        ThemeMode.entries.forEach { mode ->
            OptionRow(
                label = stringResource(mode.labelRes()),
                selected = state.themeMode == mode,
                onClick = { onAction(SettingsAction.OnThemeModeSelected(mode)) },
            )
        }

        Text(
            text = stringResource(R.string.settings_units_header),
            style = MaterialTheme.typography.titleMedium,
            modifier = Modifier.padding(
                top = HawkeyeDimens.itemSpacing,
                bottom = HawkeyeDimens.titleSpacing,
            ),
        )
        DistanceUnit.entries.forEach { unit ->
            OptionRow(
                label = stringResource(unit.labelRes()),
                selected = state.distanceUnit == unit,
                onClick = { onAction(SettingsAction.OnDistanceUnitSelected(unit)) },
            )
        }

        Text(
            text = stringResource(R.string.settings_connection_header),
            style = MaterialTheme.typography.titleMedium,
            modifier = Modifier.padding(
                top = HawkeyeDimens.itemSpacing,
                bottom = HawkeyeDimens.titleSpacing,
            ),
        )
        OutlinedTextField(
            value = state.portInput,
            onValueChange = { onAction(SettingsAction.OnListenPortChanged(it)) },
            label = { Text(stringResource(R.string.settings_listen_port_label)) },
            singleLine = true,
            isError = state.portError != null,
            supportingText = state.portError?.let { error -> { Text(error.asString()) } },
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
            modifier = Modifier.fillMaxWidth(),
        )

        Text(
            text = stringResource(R.string.settings_diagnostics_header),
            style = MaterialTheme.typography.titleMedium,
            modifier = Modifier.padding(
                top = HawkeyeDimens.itemSpacing,
                bottom = HawkeyeDimens.titleSpacing,
            ),
        )
        ToggleRow(
            label = stringResource(R.string.settings_crash_reporting_label),
            checked = state.crashReportingEnabled,
            onCheckedChange = { onAction(SettingsAction.OnCrashReportingToggled(it)) },
        )
        Text(
            text = stringResource(R.string.settings_crash_reporting_description),
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )

        Text(
            text = stringResource(R.string.settings_about_header),
            style = MaterialTheme.typography.titleMedium,
            modifier = Modifier.padding(
                top = HawkeyeDimens.itemSpacing,
                bottom = HawkeyeDimens.titleSpacing,
            ),
        )
        Text(
            text = stringResource(R.string.settings_about_label),
            // itemSpacing rather than rowVerticalPadding: this row is a bare Text, so the
            // padding is what carries it to the 48dp minimum touch target that the Material
            // radio rows above get from RadioButton.
            modifier = Modifier
                .fillMaxWidth()
                .clickable(role = Role.Button, onClick = onNavigateToAbout)
                .padding(vertical = HawkeyeDimens.itemSpacing),
        )
    }
}

@StringRes
private fun ThemeMode.labelRes(): Int = when (this) {
    ThemeMode.SYSTEM -> R.string.settings_theme_system
    ThemeMode.LIGHT -> R.string.settings_theme_light
    ThemeMode.DARK -> R.string.settings_theme_dark
}

@StringRes
private fun DistanceUnit.labelRes(): Int = when (this) {
    DistanceUnit.METRIC -> R.string.settings_unit_metric
    DistanceUnit.IMPERIAL -> R.string.settings_unit_imperial
}

@Composable
private fun OptionRow(
    label: String,
    selected: Boolean,
    onClick: () -> Unit,
) {
    Row(
        verticalAlignment = Alignment.CenterVertically,
        modifier = Modifier
            .fillMaxWidth()
            .selectable(
                selected = selected,
                onClick = onClick,
                role = Role.RadioButton,
            )
            .padding(vertical = HawkeyeDimens.rowVerticalPadding),
    ) {
        RadioButton(
            selected = selected,
            onClick = null,
        )
        Text(
            text = label,
            modifier = Modifier.padding(start = HawkeyeDimens.inlineSpacing),
        )
    }
}

@Composable
private fun ToggleRow(
    label: String,
    checked: Boolean,
    onCheckedChange: (Boolean) -> Unit,
) {
    Row(
        verticalAlignment = Alignment.CenterVertically,
        modifier = Modifier
            .fillMaxWidth()
            // Toggling from anywhere on the row, with the Switch itself inert, so the
            // control and its label are one target and one accessibility node — the same
            // arrangement OptionRow uses for its radio rows.
            .toggleable(
                value = checked,
                onValueChange = onCheckedChange,
                role = Role.Switch,
            )
            .padding(vertical = HawkeyeDimens.rowVerticalPadding),
    ) {
        Text(
            text = label,
            modifier = Modifier.weight(1f),
        )
        Switch(
            checked = checked,
            onCheckedChange = null,
        )
    }
}

@Preview(showBackground = true)
@Composable
private fun SettingsScreenPreview() {
    HawkeyeTheme {
        SettingsScreen(
            state = SettingsState(themeMode = ThemeMode.DARK, distanceUnit = DistanceUnit.IMPERIAL),
            onAction = {},
            onNavigateToAbout = {},
        )
    }
}
