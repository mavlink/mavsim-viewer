import { h } from 'vue'
import DefaultTheme from 'vitepress/theme'
import HeroVideo from './HeroVideo.vue'
import './style.css'

export default {
  extends: DefaultTheme,
  Layout() {
    return h(DefaultTheme.Layout, null, {
      'home-hero-image': () => h(HeroVideo),
    })
  },
}
