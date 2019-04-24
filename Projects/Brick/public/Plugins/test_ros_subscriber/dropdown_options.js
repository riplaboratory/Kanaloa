let topics = ["Topic 1","Topic 2", "Topic 3"];     

let sel = document.getElementById('display_dropdown1');
for(let i = 0; i < topics.length; i++) {
    let opt = document.createElement('option');
    opt.innerHTML = topics[i];
    opt.value = topics[i];
    sel.appendChild(opt);
}