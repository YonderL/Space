import { defineConfig } from 'vitepress'
import { set_sidebar } from "./utils/auto_sidebar.mjs";	// 改成自己的路径
// https://vitepress.dev/reference/site-config
export default defineConfig({
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
      { text: '文档', items: [
        { text: '技术博客', link: '/111/111.md' },
        { text: '随笔', link: '/222/111.md' },
        { text: '论文&项目', link: '/333/111.md' },
      ] }
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
      '/111/': set_sidebar('/111/'),
      '/222': set_sidebar('/222'),
      '/333': set_sidebar('/333'),
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
