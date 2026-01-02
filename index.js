// Example function to fetch the top products
async function fetchTopProducts() {
    const productsList = document.getElementById('products-list');

    // Replace this with actual API or source of product data
    const products = [
        { name: 'Bikini Razor', link: 'https://tiktok.com' },
        { name: 'Portable Projector', link: 'https://tiktok.com' },
    ];

    products.forEach(product => {
        const listItem = document.createElement('li');
        listItem.innerHTML = `<a href="${product.link}" target="_blank">${product.name}</a>`;
        productsList.appendChild(listItem);
    });
}

// Example function to fetch viral videos
async function fetchTopVideos() {
    const videosList = document.getElementById('videos-list');

    // Replace this with actual API or source of video data
    const videos = [
        { title: 'Funny Dance', link: 'https://tiktok.com' },
        { title: 'Epic Fail', link: 'https://tiktok.com' },
    ];

    videos.forEach(video => {
        const listItem = document.createElement('li');
        listItem.innerHTML = `<a href="${video.link}" target="_blank">${video.title}</a>`;
        videosList.appendChild(listItem);
    });
}

// Example function to fetch live streams
async function fetchLiveStreams() {
    const liveStreamsList = document.getElementById('live-streams-list');

    // Replace this with actual API or source of live streams data
    const liveStreams = [
        { title: 'Live Cooking Show', link: 'https://tiktok.com' },
        { title: 'Live Music Event', link: 'https://tiktok.com' },
    ];

    liveStreams.forEach(stream => {
        const listItem = document.createElement('li');
        listItem.innerHTML = `<a href="${stream.link}" target="_blank">${stream.title}</a>`;
        liveStreamsList.appendChild(listItem);
    });
}

// Call the functions to fetch the data
fetchTopProducts();
fetchTopVideos();
fetchLiveStreams();
