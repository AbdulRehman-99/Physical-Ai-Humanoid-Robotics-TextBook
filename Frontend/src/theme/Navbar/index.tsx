import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import { useThemeConfig } from '@docusaurus/theme-common';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Link from '@docusaurus/Link';
import clsx from 'clsx';
import styles from './styles.module.css';

interface NavbarItem {
  type?: string;
  position?: string;
  label?: string;
  to?: string;
  href?: string;
  sidebarId?: string;
}

interface NavbarLogo {
  alt: string;
  src: string;
}

const Navbar: React.FC = () => {
  const { navbar } = useThemeConfig();
  const title = (navbar as any).title as string;
  const logo = (navbar as any).logo as NavbarLogo;
  const items = ((navbar as any).items || []) as NavbarItem[];
  const logoUrl = useBaseUrl(logo?.src);

  const [scrolled, setScrolled] = useState(false);
  const [mobileMenuOpen, setMobileMenuOpen] = useState(false);
  const location = useLocation();

  useEffect(() => {
    const handleScroll = () => {
      setScrolled(window.scrollY > 10);
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  useEffect(() => {
    setMobileMenuOpen(false);
  }, [location]);

  // Safely filter items with null checks
  const leftItems = items && Array.isArray(items) ? items.filter(item => item?.position === 'left') : [];
  const rightItems = items && Array.isArray(items) ? items.filter(item => item?.position === 'right') : [];

  const handleLinkClick = (e: React.MouseEvent, href?: string, to?: string) => {
    if (mobileMenuOpen) {
      setMobileMenuOpen(false);
    }
  };

  const toggleMobileMenu = () => {
    setMobileMenuOpen(!mobileMenuOpen);
  };

  return (
    <nav
      className={clsx(
        'navbar',
        'navbar--fixed-top',
        styles.navbar,
        scrolled && styles.navbarScrolled
      )}
      role="navigation"
      aria-label="Main navigation"
    >
      <div className={styles.navbarContainer}>
        <Link to="/" className={styles.navbarBrand}>
          {logo && (
            <img
              src={logoUrl}
              alt={logo.alt}
              className={styles.navbarLogo}
            />
          )}
          <span className={styles.navbarTitle}>{title}</span>
        </Link>

        <button
          className={styles.mobileMenuButton}
          onClick={toggleMobileMenu}
          aria-label={mobileMenuOpen ? 'Close menu' : 'Open menu'}
          aria-expanded={mobileMenuOpen}
        >
          <span className={styles.burgerLine}></span>
          <span className={styles.burgerLine}></span>
          <span className={styles.burgerLine}></span>
        </button>

        <div className={clsx(styles.navbarNav, mobileMenuOpen && styles.navbarNavOpen)}>
          <div className={styles.navbarNavLeft}>
            {leftItems && leftItems.length > 0 && leftItems.map((item, index) => {
              if (item?.type === 'docSidebar' && item?.sidebarId) {
                // For doc sidebar items
                return (
                  <Link
                    key={item.label || `left-item-${index}`}
                    to={item.to || '/docs/intro'}
                    className={clsx(
                      styles.navbarLink,
                      styles.navbarDocLink
                    )}
                    onClick={(e) => handleLinkClick(e, undefined, item.to || '/docs/intro')}
                  >
                    {item.label}
                  </Link>
                );
              } else {
                // For regular links
                return (
                  <a
                    key={item?.label || `left-item-${index}`}
                    href={item?.href || item?.to || '#'}
                    className={styles.navbarLink}
                    onClick={(e) => handleLinkClick(e, item?.href, item?.to)}
                  >
                    {item?.label}
                  </a>
                );
              }
            })}
          </div>

          <div className={styles.navbarNavRight}>
            {rightItems && rightItems.length > 0 && rightItems.map((item, index) => (
              <a
                key={item?.label || `right-item-${index}`}
                href={item?.href || item?.to || '#'}
                className={styles.navbarLink}
                onClick={(e) => handleLinkClick(e, item?.href, item?.to)}
              >
                {item?.label}
              </a>
            ))}
          </div>
        </div>
      </div>
    </nav>
  );
};

export default Navbar;