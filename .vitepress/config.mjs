import { defineConfig } from 'vitepress'
import { set_sidebar } from "./utils/auto_sidebar.mjs";	// 改成自己的路径
import mathjax3 from 'markdown-it-mathjax3';
// https://vitepress.dev/reference/site-config
export default defineConfig({
  markdown:{
    math:true
  },
  base: '/Space/',
  title: "YonderL(LiuYang)",
  description: "A VitePress Site",
  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    logo: '/background.png',
    outlineTitle: 'On this page',
    outline: [2, 6],
    nav: [
      { text: 'Home', link: '/' },
      { text: 'About me', link: '/AboutMe' },
      { text: 'Blogs', items: [
        { text: 'ROS', link: '/blogs/ROS/阿克曼小车的URDF建模' },
        { text: 'C++', link: '/blogs/C++/Eigen3库' },
      ]},
      { text: 'Pub&Proj', items: [
        { text: 'Publication', link: '/Publication/Publication' },
        { text: 'Project', link: '/Publication/Project' }
      ]},
      { text: 'Essay', items: [
        { text: '2023', link: '/Essay/2023/11-24' },
        { text: '2024', link: '/Essay/2024/6-23' },
        { text: '2025', link: '/Essay/2025/Going' },
      ]}
    ],

    // sidebar: [
    //   {
    //     text: 'Examples',
    //     items: [
    //       { text: 'Markdown Examples', link: '/markdown-examples' },
    //       { text: 'Runtime API Examples', link: '/api-examples' }
    //     ]
    //   },
    //   {
    //     text: 'Examples2',
    //     items: [
    //       { text: 'Markdown Examples', link: '/markdown-examples' },
    //       { text: 'Runtime API Examples', link: '/api-examples' }
    //     ]
    //   }
    // ],
    sidebar: {
      '/blogs/ROS': set_sidebar('/blogs/ROS/'),
      '/blogs/C++': set_sidebar('/blogs/C++/'),
      '/Publication/': set_sidebar('/Publication/'),
      '/Essay/': set_sidebar('/Essay/'),
    },
    lastUpdatedText: 'Last Updated',

    socialLinks: [
      { icon: 'github', link: 'https://github.com/yonderl' }
    ],
    search: {
      provider: 'local',
      options: {
        translations: {
          button: {
            buttonText: 'Search',
            buttonAriaLabel: 'Search'
          },
          modal: {
            noResultsText: 'No results',
            resetButtonTitle: 'Clear query',
            footer: {
              selectText: 'Select',
              navigateText: 'Navigate',
            }
          }
        }
      }
    }
  }
})

