const nodes = document.querySelectorAll('.reveal');

const observer = new IntersectionObserver((entries) => {
  entries.forEach((entry) => {
    if (entry.isIntersecting) {
      entry.target.classList.add('visible');
      observer.unobserve(entry.target);
    }
  });
}, { threshold: 0.12 });

nodes.forEach((n, i) => {
  n.style.transitionDelay = `${Math.min(i * 50, 300)}ms`;
  observer.observe(n);
});

// Enforce silent playback for all native videos on the page.
const silentVideos = document.querySelectorAll('video');
silentVideos.forEach((v) => {
  v.muted = true;
  v.volume = 0;
  v.addEventListener('volumechange', () => {
    if (!v.muted || v.volume !== 0) {
      v.muted = true;
      v.volume = 0;
    }
  });
});
