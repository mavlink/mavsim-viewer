package com.px4.hawkeye.feature.settings.presentation

import app.cash.turbine.test
import assertk.assertThat
import assertk.assertions.isEqualTo
import assertk.assertions.isFalse
import assertk.assertions.isNotNull
import assertk.assertions.isNull
import assertk.assertions.isTrue
import com.px4.hawkeye.core.domain.DEFAULT_LIVE_PORT
import com.px4.hawkeye.feature.settings.domain.DistanceUnit
import com.px4.hawkeye.feature.settings.domain.ThemeMode
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.UnconfinedTestDispatcher
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.setMain
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

@OptIn(ExperimentalCoroutinesApi::class)
class SettingsViewModelTest {
    private val dispatcher = UnconfinedTestDispatcher()
    private lateinit var repo: FakeSettingsRepository

    @BeforeEach fun setUp() { Dispatchers.setMain(dispatcher); repo = FakeSettingsRepository() }
    @AfterEach fun tearDown() { Dispatchers.resetMain() }

    @Test
    fun `state reflects repository settings`() = runTest {
        val vm = SettingsViewModel(repo)
        vm.state.test { assertThat(awaitItem().themeMode).isEqualTo(ThemeMode.SYSTEM) }
    }

    @Test
    fun `selecting dark theme updates state`() = runTest {
        val vm = SettingsViewModel(repo)
        vm.state.test {
            awaitItem()
            vm.onAction(SettingsAction.OnThemeModeSelected(ThemeMode.DARK))
            assertThat(awaitItem().themeMode).isEqualTo(ThemeMode.DARK)
        }
    }

    @Test
    fun `selecting imperial units updates state`() = runTest {
        val vm = SettingsViewModel(repo)
        vm.state.test {
            awaitItem()
            vm.onAction(SettingsAction.OnDistanceUnitSelected(DistanceUnit.IMPERIAL))
            assertThat(awaitItem().distanceUnit).isEqualTo(DistanceUnit.IMPERIAL)
        }
    }

    @Test
    fun `valid port persists and clears error`() = runTest {
        val vm = SettingsViewModel(repo)
        vm.state.test {
            awaitItem()
            vm.onAction(SettingsAction.OnListenPortChanged("14550"))
            val s = expectMostRecentItem()
            assertThat(s.portInput).isEqualTo("14550")
            assertThat(s.portError).isNull()
        }
        assertThat(repo.settings.value.listenPort).isEqualTo(14550)
    }

    @Test
    fun `out-of-range port sets error and does not persist`() = runTest {
        val vm = SettingsViewModel(repo)
        vm.state.test {
            awaitItem()
            vm.onAction(SettingsAction.OnListenPortChanged("80"))
            assertThat(expectMostRecentItem().portError).isNotNull()
        }
        assertThat(repo.settings.value.listenPort).isEqualTo(DEFAULT_LIVE_PORT)
    }

    @Test
    fun `non-numeric port sets error and does not persist`() = runTest {
        val vm = SettingsViewModel(repo)
        vm.state.test {
            awaitItem()
            vm.onAction(SettingsAction.OnListenPortChanged("abc"))
            assertThat(expectMostRecentItem().portError).isNotNull()
        }
        assertThat(repo.settings.value.listenPort).isEqualTo(DEFAULT_LIVE_PORT)
    }

    @Test
    fun `crash reporting defaults on and opting out persists`() = runTest {
        val vm = SettingsViewModel(repo)
        vm.state.test {
            assertThat(awaitItem().crashReportingEnabled).isTrue()
            vm.onAction(SettingsAction.OnCrashReportingToggled(false))
            assertThat(awaitItem().crashReportingEnabled).isFalse()
        }
        assertThat(repo.settings.value.crashReportingEnabled).isFalse()
    }

    @Test
    fun `an in-progress invalid port is not clobbered by a later settings emission`() = runTest {
        val vm = SettingsViewModel(repo)
        vm.state.test {
            awaitItem()
            // Invalid edit: error set, raw input retained.
            vm.onAction(SettingsAction.OnListenPortChanged("99"))
            // An unrelated setting changes, re-emitting AppSettings from the repo flow.
            vm.onAction(SettingsAction.OnThemeModeSelected(ThemeMode.DARK))
            val s = expectMostRecentItem()
            assertThat(s.themeMode).isEqualTo(ThemeMode.DARK)
            // The bad input and its error survive (not reseeded from the persisted port).
            assertThat(s.portInput).isEqualTo("99")
            assertThat(s.portError).isNotNull()
        }
    }
}
