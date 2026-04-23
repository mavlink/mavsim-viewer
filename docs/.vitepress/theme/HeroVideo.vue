<template>
  <video
    class="hero-video"
    autoplay
    loop
    muted
    playsinline
    preload="metadata"
    poster="https://artifacts.px4.io/hawkeye/cover.png"
  >
    <source src="https://artifacts.px4.io/hawkeye/demo.mp4" type="video/mp4" />
  </video>
</template>

<style scoped>
.hero-video {
  width: 100%;
  height: auto;
  border-radius: 12px;
  display: block;
}
</style>
