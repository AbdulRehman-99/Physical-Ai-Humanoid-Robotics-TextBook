import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useThemeConfig } from '@docusaurus/theme-common';
import styles from './styles.module.css';

interface FooterLinkItem {
  label: string;
  href?: string;
  to?: string;
}

interface FooterColumn {
  title: string;
  items: FooterLinkItem[];
}

const Footer: React.FC = () => {
  const { footer } = useThemeConfig();
  const style = (footer as any)?.style || 'dark';
  const links = ((footer as any)?.links || []) as FooterColumn[];
  const logo = (footer as any)?.logo;
  const copyright = (footer as any)?.copyright;

  const currentYear = new Date().getFullYear();

  return (
    <footer
      className={clsx(
        styles.footer,
        style === 'dark' && styles.footerDark,
        style === 'light' && styles.footerLight
      )}
      role="contentinfo"
    >
      <div className={styles.footerContainer}>
        {logo && (
          <div className={styles.footerBrand}>
            {logo.href ? (
              <a href={logo.href} className={styles.footerLogoLink}>
                <img src={logo.src} alt={logo.alt} className={styles.footerLogo} />
                <span className={styles.footerTitle}>Physical AI & Humanoid Robotics</span>
              </a>
            ) : (
              <div className={styles.footerLogoContainer}>
                <img src={logo.src} alt={logo.alt} className={styles.footerLogo} />
                <span className={styles.footerTitle}>Physical AI & Humanoid Robotics</span>
              </div>
            )}
            <p className={styles.footerTagline}>
              A Technical Book on Physical AI & Humanoid Robotics
            </p>
          </div>
        )}

        {links && links.length > 0 && (
          <div className={styles.footerLinks}>
            {links.map((column, columnIndex) => (
              <div key={columnIndex} className={styles.footerLinkColumn}>
                <h3 className={styles.footerColumnTitle}>{column.title}</h3>
                <ul className={styles.footerLinkList}>
                  {column.items && column.items.length > 0 ? (
                    column.items.map((item, itemIndex) => (
                      <li key={itemIndex} className={styles.footerLinkItem}>
                        {item.href ? (
                          <a
                            href={item.href}
                            className={styles.footerLink}
                            target={item.href.startsWith('http') ? '_blank' : undefined}
                            rel={item.href.startsWith('http') ? 'noopener noreferrer' : undefined}
                          >
                            {item.label}
                          </a>
                        ) : item.to ? (
                          <Link to={item.to} className={styles.footerLink}>
                            {item.label}
                          </Link>
                        ) : (
                          <span className={styles.footerLink}>{item.label}</span>
                        )}
                      </li>
                    ))
                  ) : (
                    <li className={styles.footerLinkItem}>
                      <span className={styles.footerPlaceholder}>No links available</span>
                    </li>
                  )}
                </ul>
              </div>
            ))}
          </div>
        )}

        {copyright && (
          <div className={styles.footerBottom}>
            <div className={styles.footerCopyright}>
              {copyright.replace('${currentYear}', currentYear.toString())}
            </div>
            <div className={styles.footerSocial}>
              <a
                href="https://github.com/facebook/docusaurus"
                className={styles.socialLink}
                target="_blank"
                rel="noopener noreferrer"
                aria-label="GitHub"
              >
                <svg
                  className={styles.socialIcon}
                  viewBox="0 0 24 24"
                  width="20"
                  height="20"
                  fill="currentColor"
                >
                  <path d="M12 0c-6.626 0-12 5.373-12 12 0 5.302 3.438 9.8 8.207 11.387.599.111.793-.261.793-.577v-2.234c-3.338.726-4.033-1.416-4.033-1.416-.546-1.387-1.333-1.756-1.333-1.756-1.089-.745.083-.729.083-.729 1.205.084 1.839 1.237 1.839 1.237 1.07 1.834 2.807 1.304 3.492.997.107-.775.418-1.305.762-1.604-2.665-.305-5.467-1.334-5.467-5.931 0-1.311.469-2.381 1.236-3.221-.124-.303-.535-1.524.117-3.176 0 0 1.008-.322 3.301 1.23.957-.266 1.983-.399 3.003-.404 1.02.005 2.047.138 3.006.404 2.291-1.552 3.297-1.23 3.297-1.23.653 1.653.242 2.874.118 3.176.77.84 1.235 1.911 1.235 3.221 0 4.609-2.807 5.624-5.479 5.921.43.372.823 1.102.823 2.222v3.293c0 .319.192.694.801.576 4.765-1.589 8.199-6.086 8.199-11.386 0-6.627-5.373-12-12-12z" />
                </svg>
              </a>
              <a
                href="https://discordapp.com/invite/docusaurus"
                className={styles.socialLink}
                target="_blank"
                rel="noopener noreferrer"
                aria-label="Discord"
              >
                <svg
                  className={styles.socialIcon}
                  viewBox="0 0 24 24"
                  width="20"
                  height="20"
                  fill="currentColor"
                >
                  <path d="M20.317 4.37a19.791 19.791 0 0 0-4.885-1.515a.074.074 0 0 0-.079.037c-.21.375-.444.864-.608 1.25a18.27 18.27 0 0 0-5.487 0a12.64 12.64 0 0 0-.617-1.25a.077.077 0 0 0-.079-.037A19.736 19.736 0 0 0 3.677 4.37a.07.07 0 0 0-.032.027C.533 9.046-.32 13.58.099 18.057a.082.082 0 0 0 .031.057a19.9 19.9 0 0 0 5.993 3.03a.078.078 0 0 0 .084-.028a14.09 14.09 0 0 0 1.226-1.994a.076.076 0 0 0-.041-.106a13.107 13.107 0 0 1-1.872-.892a.077.077 0 0 1-.008-.128a10.2 10.2 0 0 0 .372-.292a.074.074 0 0 1 .077-.01c3.928 1.793 8.18 1.793 12.062 0a.074.074 0 0 1 .078.01c.12.098.246.198.373.292a.077.077 0 0 1-.006.127a12.299 12.299 0 0 1-1.873.892a.077.077 0 0 0-.041.107c.36.698.772 1.362 1.225 1.993a.076.076 0 0 0 .084.028a19.839 19.839 0 0 0 6.002-3.03a.077.077 0 0 0 .032-.054c.5-5.177-.838-9.674-3.549-13.66a.061.061 0 0 0-.031-.03zM8.02 15.33c-1.183 0-2.157-1.085-2.157-2.419c0-1.333.956-2.419 2.157-2.419c1.21 0 2.176 1.096 2.157 2.42c0 1.333-.956 2.418-2.157 2.418zm7.975 0c-1.183 0-2.157-1.085-2.157-2.419c0-1.333.955-2.419 2.157-2.419c1.21 0 2.176 1.096 2.157 2.42c0 1.333-.946 2.418-2.157 2.418z" />
                </svg>
              </a>
            </div>
          </div>
        )}
      </div>
    </footer>
  );
};

export default Footer;