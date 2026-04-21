import { defineConfig } from 'vitepress'

export default defineConfig({
  title: 'Hawkeye',
  description: 'Real-time 3D flight visualizer for PX4',
  base: '/Hawkeye/',
  lastUpdated: true,
  cleanUrls: true,

  head: [
    ['link', { rel: 'icon', href: '/Hawkeye/favicon.ico' }],
  ],

  themeConfig: {
    logo: '/assets/screenshot.png',

    nav: [
      { text: 'Install', link: '/installation' },
      { text: 'First SITL', link: '/first-sitl' },
      { text: 'CLI', link: '/cli' },
      { text: 'Keybinds', link: '/keybinds' },
      {
        text: 'Links',
        items: [
          { text: 'GitHub', link: 'https://github.com/PX4/Hawkeye' },
          { text: 'Releases', link: 'https://github.com/PX4/Hawkeye/releases' },
          { text: 'PX4 Docs', link: 'https://docs.px4.io' },
        ],
      },
    ],

    sidebar: [
      {
        text: 'Installation',
        link: '/installation',
      },
      {
        text: 'Getting Started',
        items: [
          { text: 'First SITL run', link: '/first-sitl' },
          { text: 'First replay', link: '/first-replay' },
          { text: 'First swarm', link: '/first-swarm' },
        ],
      },
      {
        text: 'Guides',
        items: [
          { text: 'Live SITL', link: '/sitl' },
          { text: 'ULog Replay', link: '/replay' },
          { text: 'Multi-Drone Replay', link: '/multi_drone' },
        ],
      },
      {
        text: 'Interface',
        items: [
          { text: 'HUD', link: '/hud' },
          { text: 'Cameras & Views', link: '/views' },
          { text: 'In-World Indicators', link: '/world_indicators' },
        ],
      },
      {
        text: 'Reference',
        items: [
          { text: 'Command Line', link: '/cli' },
          { text: 'Keybinds', link: '/keybinds' },
          { text: 'Position Data Tiers', link: '/position-tiers' },
          { text: 'Coordinate Systems', link: '/coordinate-systems' },
          { text: 'Data Sources', link: '/data-sources' },
          { text: 'Troubleshooting', link: '/troubleshooting' },
        ],
      },
      {
        text: 'Developer',
        items: [
          { text: 'Building from source', link: '/developer/build' },
          { text: 'Testing', link: '/developer/testing' },
        ],
      },
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/PX4/Hawkeye' },
    ],

    footer: {
      message: 'Released under the BSD-3-Clause License.',
      copyright: 'Copyright © PX4 / Dronecode Foundation',
    },

    search: {
      provider: 'local',
    },

    editLink: {
      pattern: 'https://github.com/PX4/Hawkeye/edit/main/docs/:path',
      text: 'Edit this page on GitHub',
    },
  },
})
